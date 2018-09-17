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

#include <memory>
#include <string>

#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "cybertron/time/rate.h"
#include "cybertron/time/time.h"
#include "python/wrapper/py_node.h"

using apollo::cybertron::Rate;
using apollo::cybertron::Time;
using apollo::cybertron::proto::Chatter;

using apollo::cybertron::message::PyMessageWrap;

int main(int argc, char *argv[]) {
  // init cybertron framework
  apollo::cybertron::Init("cyber_python");
  auto msgChat = std::make_shared<apollo::cybertron::proto::Chatter>();
  apollo::cybertron::PyNode node("talker");
  apollo::cybertron::PyWriter *pw =
      node.create_writer("channel/chatter", msgChat->GetTypeName());
  Rate rate(1.0);
  while (apollo::cybertron::OK()) {
    static uint64_t seq = 0;
    msgChat->set_timestamp(Time::Now().ToNanosecond());
    msgChat->set_lidar_timestamp(Time::Now().ToNanosecond());
    msgChat->set_seq(seq++);
    msgChat->set_content("Hello, apollo!");

    std::string org_data;
    msgChat->SerializeToString(&org_data);
    AINFO << "write->[ " << org_data << " ]";
    pw->write(org_data);
    AINFO << "talker sent a message!" << org_data;
    rate.Sleep();
  }
  delete pw;
  return 0;
}
