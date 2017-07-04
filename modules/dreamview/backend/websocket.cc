/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/dreamview/backend/websocket.h"

#include <functional>
#include "modules/common/log.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace apollo {
namespace dreamview {

WebsocketServer::WebsocketServer(int port, websocketpp::log::level log_level)
    : server_(), server_thread_(nullptr), connections_() {
  // Initialize the server using the ASIO transport option.
  server_.init_asio();
  // Reuse the address even if it is already bound. Under the hood it
  // sets the SO_REUSEADDR flag when opening listening TCP sockets.
  server_.set_reuse_addr(true);
  server_.set_access_channels(log_level);

  // Register the handlers.
  server_.set_validate_handler(
      std::bind(&WebsocketServer::OnAcceptConnection, this, _1));
  server_.set_fail_handler(std::bind(&WebsocketServer::OnFail, this, _1));
  server_.set_close_handler(std::bind(&WebsocketServer::OnClose, this, _1));
  server_.set_message_handler(
      std::bind(&WebsocketServer::OnMessage, this, _1, _2));

  // Start listen on the specified port.
  try {
    server_.listen(port);
  } catch (const websocketpp::exception &exception) {
    AFATAL << "Websocket failed to listen on port " << port
           << ". Reason: " << exception.what();
  }

  // Start the acceptor.
  websocketpp::lib::error_code status;
  server_.start_accept(status);
  if (status) {
    AERROR << "Failed to start the websocket server (for accepting).";
  }

  AINFO << "Websocket server ready to accept on port " << port << ".";
}

void WebsocketServer::Stop() {
  if (!server_thread_) {
    // Nothing to stop is the server thread is not active.
    return;
  }

  // Stop the acceptor.
  websocketpp::lib::error_code status;
  server_.stop_listening(status);
  if (status) {
    AFATAL << "Failed to stop the websocket server nicely. Reason: "
           << status.message();
    return;
  }

  // Clear all connections.
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for (const auto &connection : connections_) {
      server_.close(connection, websocketpp::close::status::normal,
                    "Terminating connection: server stopped.", status);
      if (status) {
        AERROR
            << "Failed to terminate the connection upon server close. Reason: "
            << status.message();
      }
    }
  }

  // Stop the server.
  server_.stop();
  server_thread_->join();
  server_thread_.reset(nullptr);
}

WebsocketServer::~WebsocketServer() {
  Stop();
}

void WebsocketServer::Run() {
  server_thread_.reset(new std::thread([this]() {
    try {
      AINFO << "SimWorld websocket server started.";
      server_.run();
    } catch (const websocketpp::exception &exception) {
      AFATAL << "SimWorkdWebsocket::Run() failed. Reason: " << exception.what();
    }
  }));
}

bool WebsocketServer::OnAcceptConnection(ConnectionHandle handle) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    connections_.insert(handle);
  }
  ServerEndpoint::connection_ptr connection = server_.get_con_from_hdl(handle);
  AINFO << "Accepted connection. Total connections: " << connections_.size();
  return true;
}

void WebsocketServer::OnFail(ConnectionHandle handle) {
  ServerEndpoint::connection_ptr connection = server_.get_con_from_hdl(handle);
  websocketpp::lib::error_code status = connection->get_ec();
  if (status) {
    AERROR << "Failed to establish a connection. Reason: " << status.message();
  }
}

void WebsocketServer::OnClose(ConnectionHandle handle) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Remove from the store of currently open connections.
    connections_.erase(handle);
  }
  AINFO << "Connection closed. Total connections: " << connections_.size();
}

bool WebsocketServer::SendData(const std::string &data) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (connections_.empty()) {
    return true;
  }

  AINFO << "Sending data to " << connections_.size() << " clients.";
  bool all_success = true;
  for (const auto &connection : connections_) {
    websocketpp::lib::error_code status;
    server_.send(connection, data, websocketpp::frame::opcode::text, status);
    if (status) {
      AWARN << "Failed to send data via websocket connection. Reason: "
            << status.message();
      all_success = false;
    }
  }

  return all_success;
}

void WebsocketServer::OnMessage(ConnectionHandle handle, MessagePtr message) {
  auto json = Json::parse(message->get_payload());
  auto type = json["type"];

  if (message_handlers_.find(type) == message_handlers_.end()) {
    AERROR << "No message handler found for message type " << type
           << ". The message will be discarded!";
    return;
  }
  message_handlers_[type](json);
}

}  // namespace dreamview
}  // namespace apollo
