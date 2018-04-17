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

#include <chrono>
#include <string>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/log.h"

using ::testing::ElementsAre;

namespace apollo {
namespace dreamview {

class MockClient {
 public:
  MockClient(const char *host, int port) {
    conn = mg_connect_websocket_client(host, port, 0, error_buffer, 100,
                                       "/websocket", "", &MockClient::OnMessage,
                                       nullptr, nullptr);
    CHECK_NOTNULL(conn);
  }

  const std::vector<std::string> &GetReceivedMessages() {
    return received_messages_;
  }

 private:
  static int OnMessage(struct mg_connection *conn, int bits, char *data,
                       size_t data_len, void *cbdata) {
    AINFO << "Get " << *data;
    received_messages_.emplace_back(data);
    return 1;
  }

  mg_connection *conn;
  char error_buffer[100];
  static std::vector<std::string> received_messages_;
};
std::vector<std::string> MockClient::received_messages_;

static WebSocketHandler handler("Test");

TEST(WebSocketTest, IntegrationTest) {
  // NOTE: Here a magic number is picked up as the port, which is not ideal but
  // in almost all cases this should be fine for a small integration test.
  CivetServer server({"listening_ports", "32695"});
  server.addWebSocketHandler("/websocket", handler);

  // Wait for a small amount of time to make sure that the server is up.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  MockClient client("localhost", 32695);

  // Wait for a small amount of time to make sure that the client is up and
  // connected.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Send 3 messages.
  for (int i = 0; i < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    handler.BroadcastData(std::to_string(i));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Check that the 3 messages are successfully received and processed.
  EXPECT_THAT(client.GetReceivedMessages(), ElementsAre("0", "1", "2"));
}

}  // namespace dreamview
}  // namespace apollo
