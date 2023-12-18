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

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "CivetServer.h"
#include "nlohmann/json.hpp"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class WebSocketHandler
 *
 * @brief The WebSocketHandler, built on top of CivetWebSocketHandler, is a
 * websocket handler that handles different types of websocket related events.
 */
class WebSocketHandler : public CivetWebSocketHandler {
  // In case of receiving fragmented message,
  // websocket opcode and accumulated data are stored.
  thread_local static unsigned char current_opcode_;
  thread_local static std::stringstream data_;

 public:
  using Json = nlohmann::json;
  using Connection = struct mg_connection;
  using MessageHandler = std::function<void(const Json &, Connection *)>;
  using ConnectionReadyHandler = std::function<void(Connection *)>;

  explicit WebSocketHandler(const std::string &name) : name_(name) {}

  /**
   * @brief Callback method for when the client intends to establish a websocket
   * connection, before websocket handshake.
   *
   * @param server the calling server
   * @param conn the connection information
   * @returns true to keep socket open, false to close it
   */
  bool handleConnection(CivetServer *server, const Connection *conn) override {
    return true;
  }

  /**
   * @brief Callback method for when websocket handshake is successfully
   * completed, and connection is ready for data exchange.
   *
   * @param server the calling server
   * @param conn the connection information
   */
  void handleReadyState(CivetServer *server, Connection *conn) override;

  /**
   * @brief Callback method for when a data frame has been received from the
   * client.
   *
   * @details In the websocket protocol, data is transmitted using a sequence of
   * frames, and each frame received invokes this callback method. Since the
   * type of opcode (text, binary, etc) is given in the first frame, this method
   * stores the opcode in a thread_local variable named current_opcode_. And
   * data from each frame is accumulated to data_ until the final fragment is
   * detected. See websocket RFC at http://tools.ietf.org/html/rfc6455, section
   * 5.4 for more protocol and fragmentation details.
   *
   * @param server the calling server
   * @param conn the connection information
   * @param bits first byte of the websocket frame, see websocket RFC at
   *             http://tools.ietf.org/html/rfc6455, section 5.2
   * @param data payload, with mask (if any) already applied.
   * @param data_len length of data
   * @returns true to keep socket open, false to close it
   */
  bool handleData(CivetServer *server, Connection *conn, int bits, char *data,
                  size_t data_len) override;

  bool handleJsonData(Connection *conn, const std::string &data);
  bool handleBinaryData(Connection *conn, const std::string &data);

  /**
   * @brief Callback method for when the connection is closed.
   *
   * @param server the calling server
   * @param conn the connection information
   */
  void handleClose(CivetServer *server, const Connection *conn) override;

  /**
   * @brief Sends the provided data to all the connected clients.
   * @param data The message string to be sent.
   */
  bool BroadcastData(const std::string &data, bool skippable = false);

  /**
   * @brief Sends the provided binary data to all the connected clients.
   * @param data The message string to be sent.
   */
  bool BroadcastBinaryData(const std::string &data, bool skippable = false);

  /**
   * @brief Sends the provided data to a specific connected client.
   *
   * @param conn The connection to send to.
   * @param data The message string to be sent.
   * @param skippable whether the data is allowed to be skipped if some other is
   * being sent to this connection.
   */
  bool SendData(Connection *conn, const std::string &data,
                bool skippable = false, int op_code = MG_WEBSOCKET_OPCODE_TEXT);

  bool SendBinaryData(Connection *conn, const std::string &data,
                      bool skippable = false);

  /**
   * @brief Add a new message handler for a message type.
   * @param type The name/key to identify the message type.
   * @param handler The function to handle the received message.
   */
  void RegisterMessageHandler(std::string type, MessageHandler handler) {
    message_handlers_[type] = handler;
  }

  /**
   * @brief Add a new handler for new connections.
   * @param handler The function to handle the new connection in ReadyState.
   */
  void RegisterConnectionReadyHandler(ConnectionReadyHandler handler) {
    connection_ready_handlers_.emplace_back(handler);
  }

 private:
  const std::string name_;

  // Message handlers keyed by message type.
  std::unordered_map<std::string, MessageHandler> message_handlers_;
  // New connection ready handlers.
  std::vector<ConnectionReadyHandler> connection_ready_handlers_;

  // The mutex guarding the connection set. We are not using read
  // write lock, as the server is not expected to get many clients
  // (connections).
  // CAVEAT: Execution section while holding this global lock should be as
  // brief as possible.
  mutable std::mutex mutex_;

  // The pool of all maintained connections. Each connection has a lock to
  // guard against simultaneous write.
  std::unordered_map<Connection *, std::shared_ptr<std::mutex>> connections_;
};

}  // namespace dreamview
}  // namespace apollo
