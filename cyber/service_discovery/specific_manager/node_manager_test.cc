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

#include "cyber/service_discovery/specific_manager/node_manager.h"

#include <memory>
#include <vector>
#include "gtest/gtest.h"

#include "cyber/common/global_data.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class NodeManagerTest : public ::testing::Test {
 protected:
  NodeManagerTest() { node_manager_ = std::make_shared<NodeManager>(); }
  virtual ~NodeManagerTest() { node_manager_->Shutdown(); }

  virtual void SetUp() {}

  virtual void TearDown() {}

  std::shared_ptr<NodeManager> node_manager_;
};

TEST_F(NodeManagerTest, node_change) {
  EXPECT_FALSE(node_manager_->HasNode("node"));

  // node join
  RoleAttributes role_attr;
  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_WRITER));
  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_NODE));

  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_WRITER));
  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_NODE));

  EXPECT_TRUE(node_manager_->HasNode("node"));

  // node leave
  EXPECT_FALSE(node_manager_->Leave(role_attr, RoleType::ROLE_WRITER));
  EXPECT_FALSE(node_manager_->Leave(role_attr, RoleType::ROLE_NODE));

  EXPECT_FALSE(node_manager_->HasNode("node"));
}

TEST_F(NodeManagerTest, topo_module_leave) {
  RoleAttributes role_attr;
  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_NODE));
  EXPECT_TRUE(node_manager_->HasNode("node"));
}

TEST_F(NodeManagerTest, add_and_remove_change_listener) {
  bool recv_flag = false;
  auto conn = node_manager_->AddChangeListener(
      [&recv_flag](const ChangeMsg& msg) { recv_flag = true; });

  RoleAttributes role_attr;
  role_attr.set_host_name("caros");
  role_attr.set_process_id(1024);
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_NODE));
  EXPECT_TRUE(recv_flag);

  node_manager_->RemoveChangeListener(conn);

  recv_flag = false;
  EXPECT_FALSE(node_manager_->Leave(role_attr, RoleType::ROLE_NODE));
  EXPECT_FALSE(recv_flag);
}

TEST_F(NodeManagerTest, has_node) {
  RoleAttributes role_attr;
  role_attr.set_host_name("caros");
  role_attr.set_process_id(1024);
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_NODE));

  EXPECT_TRUE(node_manager_->HasNode("node"));
  EXPECT_FALSE(node_manager_->HasNode("node11"));
}

TEST_F(NodeManagerTest, get_nodes) {
  RoleAttributes role_attr;
  role_attr.set_host_name("caros");
  role_attr.set_process_id(1024);
  role_attr.set_node_name("node_1");
  uint64_t node_id = common::GlobalData::RegisterNode("node_1");
  role_attr.set_node_id(node_id);

  EXPECT_FALSE(node_manager_->Join(role_attr, RoleType::ROLE_NODE));

  std::vector<RoleAttributes> attr_nodes;
  node_manager_->GetNodes(&attr_nodes);
  EXPECT_EQ(attr_nodes.size(), 1);
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
