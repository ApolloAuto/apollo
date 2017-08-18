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

#include "modules/dreamview/backend/websocket/websocket.h"

#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace dreamview {

void WebSocketHandler::handleReadyState(CivetServer *server, Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    connections_[conn];  // This is acutally an insertion as the mutex is
                         // neither copyable nor movable.
  }
  AINFO << "Accepted connection. Total connections: " << connections_.size();
}

void WebSocketHandler::handleClose(CivetServer *server,
                                   const Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Remove from the store of currently open connections.
    connections_.erase(const_cast<Connection *>(conn));
  }
  AINFO << "Connection closed. Total connections: " << connections_.size();
}

bool WebSocketHandler::SendData(const std::string &data) {
  std::vector<std::pair<Connection *, std::mutex *>> connections_to_send;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (connections_.empty()) {
      return true;
    }
    for (auto &kv : connections_) {
      Connection *conn = kv.first;
      std::mutex *lock = &kv.second;
      // If another thread is writing via this connection, we just skip this
      // round of broadcasting.
      if (lock->try_lock()) {
        connections_to_send.emplace_back(conn, lock);
      }
    }
  }

  bool all_success = true;
  for (auto &pair : connections_to_send) {
    Connection *conn = pair.first;
    std::mutex *lock = pair.second;

    if (!SendData(data, conn)) {
      all_success = false;
    }

    lock->unlock();
  }

  return all_success;
}

bool WebSocketHandler::SendData(const std::string &data, Connection *conn) {
  if (connections_.find(conn) == connections_.end()) {
    AERROR << "Trying to send to an uncached connection, skipping.";
    return false;
  }

  // Lock the connection while sending.
  connections_[conn].try_lock();
  int ret = mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, data.c_str(),
                               data.size());
  connections_[conn].unlock();

  if (ret != static_cast<int>(data.size())) {
    // Determine error message based on return value.
    std::string msg;
    if (ret == 0) {
      msg = "Connection Closed";
    } else if (ret < 0) {
      msg = "Send Error";
    } else {
      msg = apollo::common::util::StrCat("Expect to send ", data.size(),
                                         " bytes. But sent ", ret, " bytes");
    }
    AWARN << "Failed to send data via websocket connection. Reason: " << msg;
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
    AERROR << "No message handler found for message type " << type
           << ". The message will be discarded!";
    return true;
  }
  message_handlers_[type](json, conn);
  return true;
}

}  // namespace dreamview
}  // namespace apollo
