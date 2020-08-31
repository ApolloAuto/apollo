/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "cyber/examples/proto/examples.pb.h"

#include "cyber/cyber.h"

using apollo::cyber::examples::proto::Driver;

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("start_node"));
  auto server = node->CreateService<Driver, Driver>(
      "test_server", [](const std::shared_ptr<Driver>& request,
                        std::shared_ptr<Driver>& response) {
        AINFO << "server: i am driver server";
        static uint64_t id = 0;
        ++id;
        response->set_msg_id(id);
        response->set_timestamp(0);
      });
  auto client = node->CreateClient<Driver, Driver>("test_server");
  auto driver_msg = std::make_shared<Driver>();
  driver_msg->set_msg_id(0);
  driver_msg->set_timestamp(0);
  while (apollo::cyber::OK()) {
    auto res = client->SendRequest(driver_msg);
    if (res != nullptr) {
      AINFO << "client: responese: " << res->ShortDebugString();
    } else {
      AINFO << "client: service may not ready.";
    }
    sleep(1);
  }

  apollo::cyber::WaitForShutdown();
  return 0;
}
