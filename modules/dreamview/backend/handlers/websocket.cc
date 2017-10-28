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

#include "modules/dreamview/backend/handlers/websocket.h"

#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::StrCat;

void WebSocketHandler::handleReadyState(CivetServer *server, Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    connections_.emplace(conn, std::make_shared<std::mutex>());
    AINFO << "Accepted connection. Total connections: " << connections_.size();
  }
}

void WebSocketHandler::handleClose(CivetServer *server,
                                   const Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // Remove from the store of currently open connections. Copy the mutex out
    // so that it won't be reclaimed during map.erase().
    Connection *connection = const_cast<Connection *>(conn);
    std::shared_ptr<std::mutex> connection_lock = connections_[connection];
    {
      std::unique_lock<std::mutex> lock(*connection_lock);
      connections_.erase(connection);
    }
    AINFO << "Connection closed. Total connections: " << connections_.size();
  }
}

bool WebSocketHandler::BroadcastData(const std::string &data, bool skippable) {
  std::vector<Connection *> connections_to_send;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (connections_.empty()) {
      return true;
    }
    for (auto &kv : connections_) {
      Connection *conn = kv.first;
      connections_to_send.push_back(conn);
    }
  }

  bool all_success = true;
  for (Connection *conn : connections_to_send) {
    if (!SendData(conn, data, skippable)) {
      all_success = false;
    }
  }

  return all_success;
}

bool WebSocketHandler::SendData(Connection *conn, const std::string &data,
                                bool skippable) {
  std::shared_ptr<std::mutex> connection_lock;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (connections_.find(conn) == connections_.end()) {
      AERROR << "Trying to send to an uncached connection, skipping.";
      return false;
    }
    // Copy the lock so that it still exists if the connection is closed after
    // this block.
    connection_lock = connections_[conn];
  }

  // Lock the connection while sending.
  if (!connection_lock->try_lock()) {
    // Skip sending data if:
    // 1. Data is skippable according to sender and there's higher priority data
    // being sent.
    // 2. The connection has been closed.
    if (skippable) {
      return false;
    } else {
      connection_lock->lock();  // Block to acquire the lock.
      if (connections_.find(conn) == connections_.end()) {
        return false;
      }
    }
  }
  // Note that while we are holding the connection lock, the connection won't be
  // closed and removed.
  int ret;
  PERF_BLOCK(StrCat("Writing ", data.size(), " bytes via websocket took"),
             0.1) {
    ret = mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, data.c_str(),
                             data.size());
  }
  connection_lock->unlock();

  if (ret != static_cast<int>(data.size())) {
    // Determine error message based on return value.
    std::string msg;
    if (ret == 0) {
      msg = "Connection Closed";
    } else if (ret < 0) {
      msg = "Send Error";
    } else {
      msg = StrCat("Expect to send ", data.size(), " bytes. But sent ", ret,
                   " bytes");
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

  Json json;
  try {
    json = Json::parse(data, data + data_len);
  } catch (const std::exception &e) {
    AERROR << "Failed to parse JSON data: " << e.what();
    return false;
  }

  if (json.find("type") == json.end()) {
    AERROR << "Received JSON data without type field: " << json;
    return true;
  }

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
