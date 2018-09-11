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

#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/cybertron.h"
#include "cybertron/time/time.h"
#include "cybertron/topology/topology.h"
#include "cybertron/transport/common/identity.h"

using apollo::cybertron::Time;
using apollo::cybertron::common::GlobalData;
using apollo::cybertron::proto::ChangeMsg;
using apollo::cybertron::proto::ChangeType;
using apollo::cybertron::proto::OperateType;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::proto::RoleType;
using apollo::cybertron::topology::RoleBase;
using apollo::cybertron::topology::RolePtr;
using apollo::cybertron::topology::Topology;
using apollo::cybertron::transport::Identity;

void TopoCallback(const ChangeMsg& msg) {
  uint64_t recv_timestamp = Time::Now().ToNanosecond();
  uint64_t send_timestamp = msg.timestamp();
  uint64_t latency = recv_timestamp - send_timestamp;
  AINFO << latency << " " << msg.ShortDebugString();
}

int main(int argc, char* argv[]) {
  apollo::cybertron::Init(argv[0]);

  auto node_mgr = Topology::Instance()->node_manager();
  node_mgr->AddChangeListener(std::bind(TopoCallback, std::placeholders::_1));

  auto channel_mgr = Topology::Instance()->channel_manager();
  channel_mgr->AddChangeListener(
      std::bind(TopoCallback, std::placeholders::_1));

  auto service_mgr = Topology::Instance()->service_manager();
  service_mgr->AddChangeListener(
      std::bind(TopoCallback, std::placeholders::_1));

  apollo::cybertron::WaitForShutdown();

  return 0;
}
