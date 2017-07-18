/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/dreamview/backend/websocket.h"

#include <sstream>
#include <vector>
#include "glog/logging.h"

namespace apollo {
namespace dreamview {

void WebSocketHandler::handleReadyState(CivetServer *server, Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    connections_.insert(conn);
  }
  LOG(INFO) << "Accepted connection. Total connections: "
            << connections_.size();
}

void WebSocketHandler::handleClose(CivetServer *server,
                                   const Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Remove from the store of currently open connections.
    connections_.erase(const_cast<Connection *>(conn));
  }
  LOG(INFO) << "Connection closed. Total connections: " << connections_.size();
}

bool WebSocketHandler::SendData(const std::string &data) {
  std::vector<Connection *> connections_to_send;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (connections_.empty()) {
      return true;
    }
    for (Connection *conn : connections_) {
      connections_to_send.push_back(conn);
    }
  }

  bool all_success = true;
  for (Connection *conn : connections_to_send) {
    if (!SendData(data, conn)) {
      all_success = false;
    }
  }

  return all_success;
}

bool WebSocketHandler::SendData(const std::string &data, Connection *conn) {
  int ret = mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, data.c_str(),
                               data.size());
  if (ret != static_cast<int>(data.size())) {
    // Determine error message based on return value.
    std::string msg;
    if (ret == 0) {
      msg = "Connection Closed";
    } else if (ret < 0) {
      msg = "Send Error";
    } else {
      std::ostringstream os;
      os << "Expect to send " << data.size() << " bytes. But sent " << ret
         << " bytes";
      msg = os.str();
    }
    LOG(WARNING) << "Failed to send data via websocket connection. Reason: "
                 << msg;
    return false;
  }

  return true;
}

bool WebSocketHandler::handleData(CivetServer *server, Connection *conn,
                                  int bits, char *data, size_t data_len) {
  // Ignore connection close request.
  if ((bits & 0x0F) == WEBSOCKET_OPCODE_CONNECTION_CLOSE) {
    return false;
  }

  auto json = Json::parse(std::string(data, data_len));
  auto type = json["type"];

  if (message_handlers_.find(type) == message_handlers_.end()) {
    LOG(ERROR) << "No message handler found for message type " << type
               << ". The message will be discarded!";
    return true;
  }
  message_handlers_[type](json, conn);
  return true;
}

}  // namespace dreamview
}  // namespace apollo
