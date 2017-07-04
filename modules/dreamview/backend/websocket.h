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

/**
 * @file
 */

#ifndef MODULES_DREAMVIEW_BACKEND_WEBSOCKET_H_
#define MODULES_DREAMVIEW_BACKEND_WEBSOCKET_H_

#include <unordered_map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "third_party/json/json.hpp"
#include "websocketpp/client.hpp"
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/server.hpp"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class WebsocketServer
 *
 * @brief The WebsocketServer, built on top of the websocketpp library,
 * represents a server endpoint that follows the websocket protocol.
 * When started, it will launch an acceptor to accept incoming connections,
 * and manages a pool of connections when running. Each connection is between
 * the server and a client endpoint. The SendData() method is used to push data
 * to all the connected clients.
 */
class WebsocketServer {
 public:
  using ConnectionHandle = websocketpp::connection_hdl;
  using ConnectionSet =
      std::set<ConnectionHandle, std::owner_less<ConnectionHandle>>;
  using ServerEndpoint = websocketpp::server<websocketpp::config::asio>;
  using MessagePtr = websocketpp::config::asio_client::message_type::ptr;
  using Json = nlohmann::json;
  using MessageHandler = std::function<void(const Json &)>;

  /// With the DEFAULT_LOG_LEVEL setting, websocketpp library logs the event
  /// when a client gets connected, disconnected, or pushed with data payload.
  static constexpr websocketpp::log::level DEFAULT_LOG_LEVEL =
      (websocketpp::log::alevel::connect |
       websocketpp::log::alevel::disconnect |
       websocketpp::log::alevel::message_payload);

  /// With the NO_LOG configuration, nothing will be logged by the websocketpp
  /// library.
  static constexpr websocketpp::log::level NO_LOG =
      websocketpp::log::alevel::none;

  /**
   * @brief Constructor in which you can specify the log level.
   * @param port The port on which the server will be launched.
   * @param log_level The log level for which the events will be logged by the
   * websocketpp library, could use DEFAULT_LOG_LEVEL or NO_LOG defined above.
   */
  WebsocketServer(int port, websocketpp::log::level log_level);

  /**
   * @brief Constructor with the default log level.
   * @param port The port on which the server will be launched.
   */
  explicit WebsocketServer(int port)
      : WebsocketServer(port, DEFAULT_LOG_LEVEL) {}

  ~WebsocketServer();

  /**
   * @brief Call this method to actually run the server. This method will
   * create a BACKGROUND THREAD for the server listeners and event handlers.
   *
   * NOTE: This method does not block.
   */
  void Run();

  /**
   * @brief This method will stop listening for new connections, close all the
   * current connections, and finally stop the BACKGROUND THREAD of the server.
   */
  void Stop();

  /**
   * @brief Sends the provided data to all the connected clients.
   * @param data The message string to be sent.
   */
  bool SendData(const std::string &data);

  /**
   * @brief Add a new message handler for a message type.
   * @param type The name/key to identify the message type.
   * @param handler The function to handle the received message.
   */
  void RegisterMessageHandler(std::string type, MessageHandler handler) {
    message_handlers_[type] = handler;
  }

 private:
  // The server endpoint.
  ServerEndpoint server_;

  // The thread for the server (listeners and handlers).
  std::unique_ptr<std::thread> server_thread_;

  // The pool of all maintained connections.
  ConnectionSet connections_;

  // Message handlers keyed by message type.
  std::unordered_map<std::string, MessageHandler> message_handlers_;

  // The mutex guarding the connection set. We are not using read
  // write lock, as the server is not expected to get many clients
  // (connections).
  mutable std::mutex mutex_;

  // OnAcceptConnection() will be called upon connection request
  // (handshake) from a new client. It returns true if the connection
  // is accepted, or false if it is rejected.
  bool OnAcceptConnection(ConnectionHandle handle);

  // OnFail() will be called when a websocket connection attempt by
  // client fails.
  void OnFail(ConnectionHandle handle);

  // OnClose() will be called when a client gets disconnected, either
  // proactively or passively.
  void OnClose(ConnectionHandle handle);

  // OnMessage will be called when the server receives a message from
  // client, it will dispatch the message to the corresponding handler
  // based on the message type.
  void OnMessage(ConnectionHandle handle, MessagePtr message);
};

}  // namespace dreamview
}  // namespace apollo

#endif /* MODULES_DREAMVIEW_BACKEND_WEBSOCKET_H_ */
