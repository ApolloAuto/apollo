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

#include "gtest/gtest.h"

#include "cybertron/common/global_data.h"
#include "cybertron/topology/manager/node_manager.h"
#include "cybertron/transport/rtps/participant.h"

namespace apollo {
namespace cybertron {
namespace topology {

TEST(NodeManagerTest, constructor) {
  NodeManager node_manager;
  std::vector<RoleAttributes> nodes;
  EXPECT_TRUE(nodes.empty());

  node_manager.GetNodes(&nodes);
  EXPECT_TRUE(nodes.empty());
}

TEST(NodeManagerTest, init_and_shutdown) {
  NodeManager node_manager;
  EXPECT_FALSE(node_manager.Init(nullptr));
  node_manager.Shutdown();

  auto participant =
      std::make_shared<transport::Participant>("caros+1024", 11511);
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));
  // repeated call
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));
  node_manager.Shutdown();
}

TEST(NodeManagerTest, node_change) {
  auto participant =
      std::make_shared<transport::Participant>("caros+1024", 11511);
  NodeManager node_manager;
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));
  EXPECT_FALSE(node_manager.HasNode("node"));

  // node join
  RoleAttributes role_attr;
  EXPECT_FALSE(node_manager.Join(role_attr, RoleType::ROLE_WRITER));
  EXPECT_FALSE(node_manager.Join(role_attr, RoleType::ROLE_NODE));

  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_FALSE(node_manager.Join(role_attr, RoleType::ROLE_WRITER));
  EXPECT_TRUE(node_manager.Join(role_attr, RoleType::ROLE_NODE));

  EXPECT_TRUE(node_manager.HasNode("node"));

  // node leave
  EXPECT_FALSE(node_manager.Leave(role_attr, RoleType::ROLE_WRITER));
  EXPECT_TRUE(node_manager.Leave(role_attr, RoleType::ROLE_NODE));

  EXPECT_FALSE(node_manager.HasNode("node"));

  node_manager.Shutdown();
}

TEST(NodeManagerTest, topo_module_leave) {
  auto participant =
      std::make_shared<transport::Participant>("caros+1024", 11511);
  NodeManager node_manager;
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));

  RoleAttributes role_attr;
  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_TRUE(node_manager.Join(role_attr, RoleType::ROLE_NODE));
  EXPECT_TRUE(node_manager.HasNode("node"));

  node_manager.OnTopoModuleLeave(role_attr.host_name(), role_attr.process_id());

  EXPECT_FALSE(node_manager.HasNode("node"));
  node_manager.Shutdown();
}

TEST(NodeManagerTest, add_and_remove_change_listener) {
  auto participant =
      std::make_shared<transport::Participant>("caros+1024", 11511);
  NodeManager node_manager;
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));

  bool recv_flag = false;
  auto conn = node_manager.AddChangeListener(
      [&recv_flag](const ChangeMsg& msg) { recv_flag = true; });

  RoleAttributes role_attr;
  role_attr.set_host_name("caros");
  role_attr.set_process_id(1024);
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_TRUE(node_manager.Join(role_attr, RoleType::ROLE_NODE));
  EXPECT_TRUE(recv_flag);

  node_manager.RemoveChangeListener(conn);

  recv_flag = false;
  EXPECT_TRUE(node_manager.Leave(role_attr, RoleType::ROLE_NODE));
  EXPECT_FALSE(recv_flag);

  node_manager.Shutdown();
}

TEST(NodeManagerTest, has_node) {
  auto participant =
      std::make_shared<transport::Participant>("caros+1024", 11511);
  NodeManager node_manager;
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));

  RoleAttributes role_attr;
  role_attr.set_host_name("caros");
  role_attr.set_process_id(1024);
  role_attr.set_node_name("node");
  uint64_t node_id = common::GlobalData::RegisterNode("node");
  role_attr.set_node_id(node_id);

  EXPECT_TRUE(node_manager.Join(role_attr, RoleType::ROLE_NODE));

  EXPECT_TRUE(node_manager.HasNode("node"));
  EXPECT_FALSE(node_manager.HasNode("node11"));

  node_manager.Shutdown();
}

TEST(NodeManagerTest, get_nodes) {
  auto participant =
      std::make_shared<transport::Participant>("caros+1024", 11511);
  NodeManager node_manager;
  EXPECT_TRUE(node_manager.Init(participant->fastrtps_participant()));

  RoleAttributes role_attr;
  role_attr.set_host_name("caros");
  role_attr.set_process_id(1024);
  role_attr.set_node_name("node_1");
  uint64_t node_id = common::GlobalData::RegisterNode("node_1");
  role_attr.set_node_id(node_id);

  EXPECT_TRUE(node_manager.Join(role_attr, RoleType::ROLE_NODE));

  std::vector<RoleAttributes> attr_nodes;
  node_manager.GetNodes(&attr_nodes);
  EXPECT_EQ(attr_nodes.size(), 1);

  node_manager.Shutdown();
}

}  // namespace topology
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
