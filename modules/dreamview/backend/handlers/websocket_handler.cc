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

#include "modules/dreamview/backend/handlers/websocket_handler.h"

#include <cerrno>
#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::util::ContainsKey;
using apollo::common::util::StrCat;

void WebSocketHandler::handleReadyState(CivetServer *server, Connection *conn) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    connections_.emplace(conn, std::make_shared<std::mutex>());
    AINFO << name_ << ": Accepted connection. Total connections: "
          << connections_.size();
  }

  // Trigger registered new connection handlers.
  for (const auto handler : connection_ready_handlers_) {
    handler(conn);
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
    AINFO << name_
          << ": Connection closed. Total connections: " << connections_.size();
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

bool WebSocketHandler::SendBinaryData(Connection *conn, const std::string &data,
                                      bool skippable) {
  return SendData(conn, data, skippable, WEBSOCKET_OPCODE_BINARY);
}

bool WebSocketHandler::SendData(Connection *conn, const std::string &data,
                                bool skippable, int op_code) {
  std::shared_ptr<std::mutex> connection_lock;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ContainsKey(connections_, conn)) {
      AERROR << name_
             << ": Trying to send to an uncached connection, skipping.";
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
      std::unique_lock<std::mutex> lock(mutex_);
      if (!ContainsKey(connections_, conn)) {
        return false;
      }
    }
  }

  // Note that while we are holding the connection lock, the connection won't be
  // closed and removed.
  int ret;
  PERF_BLOCK(
      StrCat(name_, ": Writing ", data.size(), " bytes via websocket took"),
      0.1) {
    ret = mg_websocket_write(conn, op_code, data.c_str(), data.size());
  }
  connection_lock->unlock();

  if (ret != static_cast<int>(data.size())) {
    // When data is empty, the header length (2) is returned.
    if (data.size() == 0 && ret == 2) {
      return true;
    }

    // Determine error message based on return value.
    std::string msg;
    if (ret == 0) {
      msg = "Connection Closed";
    } else if (ret < 0) {
      msg = "Send Error: " + std::string(std::strerror(errno));
    } else {
      msg = StrCat("Expect to send ", data.size(), " bytes. But sent ", ret,
                   " bytes");
    }
    AWARN << name_
          << ": Failed to send data via websocket connection. Reason: " << msg;
    return false;
  }

  return true;
}

thread_local unsigned char WebSocketHandler::current_opcode_ = 0x00;
thread_local std::stringstream WebSocketHandler::data_;

bool WebSocketHandler::handleData(CivetServer *server, Connection *conn,
                                  int bits, char *data, size_t data_len) {
  // Ignore connection close request.
  if ((bits & 0x0F) == WEBSOCKET_OPCODE_CONNECTION_CLOSE) {
    return false;
  }

  data_.write(data, data_len);
  if (current_opcode_ == 0x00) {
    current_opcode_ = bits & 0x7f;
  }

  bool result = true;

  // The FIN bit (the left most significant bit) is used to indicates
  // the final fragment in a message. Note, the first fragment MAY
  // also be the final fragment.
  bool is_final_fragment = bits & 0x80;
  if (is_final_fragment) {
    switch (current_opcode_) {
      case WEBSOCKET_OPCODE_TEXT:
        result = handleJsonData(conn, data_.str());
        break;
      case WEBSOCKET_OPCODE_BINARY:
        result = handleBinaryData(conn, data_.str());
        break;
      default:
        AERROR << name_ << ": Unknown WebSocket bits flag: " << bits;
        break;
    }

    // reset opcode and data
    current_opcode_ = 0x00;
    data_.clear();
    data_.str(std::string());
  }

  return result;
}

bool WebSocketHandler::handleJsonData(Connection *conn,
                                      const std::string &data) {
  Json json;
  try {
    json = Json::parse(data.begin(), data.end());
  } catch (const std::exception &e) {
    AERROR << "Failed to parse JSON data: " << e.what();
    return false;
  }

  if (!ContainsKey(json, "type")) {
    AERROR << "Received JSON data without type field: " << json;
    return true;
  }

  auto type = json["type"];
  if (!ContainsKey(message_handlers_, type)) {
    AERROR << "No message handler found for message type " << type
           << ". The message will be discarded!";
    return true;
  }
  message_handlers_[type](json, conn);
  return true;
}

bool WebSocketHandler::handleBinaryData(Connection *conn,
                                        const std::string &data) {
  auto type = "Binary";
  message_handlers_[type](data, conn);
  return true;
}

}  // namespace dreamview
}  // namespace apollo
