/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/py_wrapper/py_cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::proto::Chatter;

using apollo::cyber::message::PyMessageWrap;
apollo::cyber::PyService *server = nullptr;
int service_callback(const char *channel_name) {
  if (server == nullptr) {
    AERROR << "server is null.";
    return -1;
  }

  AINFO << "server recv channelname ->[ " << channel_name << " ]";

  Chatter chat;
  std::string res = server->read();
  chat.ParseFromString(res);
  AINFO << "server read: responese: " << chat.ShortDebugString();

  Chatter driver_msg;
  static uint64_t id = 0;
  ++id;
  driver_msg.set_seq(id);
  driver_msg.set_timestamp(88);
  std::string org_data;
  driver_msg.SerializeToString(&org_data);
  server->write(org_data);
  return 0;
}

int main(int argc, char *argv[]) {
  apollo::cyber::Init(argv[0]);
  apollo::cyber::PyNode node("start_node");
  Chatter driver_msg;
  driver_msg.set_seq(99);
  driver_msg.set_timestamp(88);

  server = node.create_service("test_service", driver_msg.GetTypeName());
  auto client = node.create_client("test_service", driver_msg.GetTypeName());
  server->register_func(service_callback);
  while (apollo::cyber::OK()) {
    std::string org_data;
    driver_msg.SerializeToString(&org_data);
    AINFO << "=========================";
    std::string res = client->send_request(org_data);
    if (!res.empty()) {
      Chatter chat;
      chat.ParseFromString(res);
      AINFO << "client: responese: " << chat.ShortDebugString();
    } else {
      AINFO << "client: service may not ready.";
    }
    sleep(1);
  }

  apollo::cyber::WaitForShutdown();
  return 0;
}
