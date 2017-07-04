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

#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/log.h"
#include "websocketpp/client.hpp"
#include "websocketpp/config/asio_no_tls_client.hpp"

using ::testing::ElementsAre;

namespace apollo {
namespace dreamview {

using ClientEndpoint = websocketpp::client<websocketpp::config::asio_client>;
using MessagePtr = websocketpp::config::asio_client::message_type::ptr;
using ConnectionHandle = websocketpp::connection_hdl;
using std::placeholders::_1;
using std::placeholders::_2;

class MockClient {
 public:
  MockClient(const std::string &host, int port)
      : client_(),
        uri_("ws://" + host + ":" + std::to_string(port)),
        received_messages_() {}

  void Run() {
    try {
      client_.set_access_channels(websocketpp::log::alevel::none);
      client_.init_asio();
      client_.set_message_handler(
          std::bind(&MockClient::OnMessage, this, _1, _2));
      websocketpp::lib::error_code status;

      // Make the connection to the server.
      ClientEndpoint::connection_ptr connection =
          client_.get_connection(uri_, status);
      if (status) {
        AERROR << "Could not make connection to the server " << uri_
               << ". Reason: " << status.message();
        return;
      }
      client_.connect(connection);
      client_.run();
    } catch (const websocketpp::exception &exception) {
      AERROR << exception.what();
    }
  }

  void Stop() {
    client_.stop();
  }

  const std::vector<std::string> &GetReceivedMessages() {
    return received_messages_;
  }

 private:
  void OnMessage(ConnectionHandle handle, MessagePtr message) {
    LOG(INFO) << "Get " << message->get_payload();
    received_messages_.push_back(message->get_payload());
  }

  ClientEndpoint client_;
  std::string uri_;
  std::vector<std::string> received_messages_;
};

TEST(WebsocketServerTest, IntegrationTest) {
  // NOTE: Here a magic number is picked up as the port, which is not
  // ideal but in almost all cases this should be fine for a small
  // integration test.
  int port = 32695;
  WebsocketServer websocket(port, WebsocketServer::NO_LOG);
  websocket.Run();

  // Wait for a small amount of time to make sure that the server is
  // up.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  MockClient client("localhost", port);
  std::thread client_thread([&client]() { client.Run(); });

  // Wait for a small amount of time to make sure that the client is
  // up and connected.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Send 3 messages.
  for (int i = 0; i < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    websocket.SendData(std::to_string(i));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Stop the clients.
  client.Stop();
  client_thread.join();

  // Check that the 3 messages are successfully received and
  // processed.
  EXPECT_THAT(client.GetReceivedMessages(), ElementsAre("0", "1", "2"));
}

}  // namespace dreamview
}  // namespace apollo
