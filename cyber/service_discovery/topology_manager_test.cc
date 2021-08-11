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

#include "cyber/service_discovery/topology_manager.h"

#include <memory>

#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class TopologyTest : public ::testing::Test {
 protected:
  TopologyTest() { topology_ = TopologyManager::Instance(); }
  virtual ~TopologyTest() { topology_->Shutdown(); }

  virtual void SetUp() {}

  virtual void TearDown() {}

  TopologyManager* topology_;
};

TEST_F(TopologyTest, add_and_remove_change_listener) {
  proto::RoleAttributes attr;
  attr.set_host_name("");
  attr.set_process_id(0);

  // add change listener
  auto conn =
      topology_->AddChangeListener([&attr](const ChangeMsg& change_msg) {
        if (change_msg.change_type() == ChangeType::CHANGE_PARTICIPANT &&
            change_msg.operate_type() == OperateType::OPT_JOIN &&
            change_msg.role_type() == RoleType::ROLE_PARTICIPANT) {
          attr.CopyFrom(change_msg.role_attr());
        }
      });

  // remove change listener
  topology_->RemoveChangeListener(conn);
}

TEST_F(TopologyTest, get_manager) {
  auto node_manager = topology_->node_manager();
  EXPECT_NE(node_manager, nullptr);

  auto channel_manager = topology_->channel_manager();
  EXPECT_NE(channel_manager, nullptr);

  auto service_manager = topology_->service_manager();
  EXPECT_NE(service_manager, nullptr);
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
