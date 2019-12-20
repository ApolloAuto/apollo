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

#include "cyber/service_discovery/container/single_value_warehouse.h"

#include <utility>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

using proto::RoleAttributes;

TEST(SingleValueWarehouseTest, test1) {
  { SingleValueWarehouse sh; }

  {
    SingleValueWarehouse sh;
    EXPECT_EQ(sh.Size(), 0);

    RoleAttributes attr;
    attr.set_host_name("caros");
    attr.set_process_id(12345);

    int i = 0, id = 0;
    {
      attr.set_node_id(i);

      auto role = std::make_shared<RoleBase>();
      role->set_attributes(attr);
      sh.Add(id++, role);

      role = std::make_shared<RoleBase>();
      role->set_attributes(attr);
      sh.Add(id++, role);

      role = std::make_shared<RoleBase>();
      role->set_attributes(attr);
      sh.Add(id++, role);
    }
    {
      auto role = std::make_shared<RoleBase>();
      attr.set_node_id(i++);
      role->set_attributes(attr);
      sh.Add(id++, role);
    }
    {
      auto role = std::make_shared<RoleBase>();
      attr.set_node_id(i);
      role->set_attributes(attr);
      sh.Add(id++, role);
    }
    {
      attr.set_node_id(i++);
      auto role = std::make_shared<RoleWriter>();
      role->set_attributes(attr);
      role->set_timestamp_ns(54321);
      sh.Add(id++, role);

      role = std::make_shared<RoleWriter>();
      role->set_attributes(attr);
      role->set_timestamp_ns(54321);
      sh.Add(id++, role);
    }
    {
      auto role = std::make_shared<RoleBase>();
      attr.set_node_id(i);
      role->set_attributes(attr);
      sh.Add(id++, role);
    }
    {
      auto role = std::make_shared<RoleWriter>();
      attr.set_node_id(i);
      role->set_attributes(attr);
      role->set_timestamp_ns(54321);
      sh.Add(id++, role);
    }

    EXPECT_EQ(sh.Size(), 9);

    std::vector<RoleAttributes> roles_attr;
    std::vector<RolePtr> roles;

    sh.GetAllRoles(&roles_attr);
    sh.GetAllRoles(&roles);

    EXPECT_EQ(roles.size(), roles_attr.size());
    EXPECT_EQ(roles.size(), sh.Size());

    roles.clear();
    roles_attr.clear();

    attr.set_node_id(0);

    EXPECT_TRUE(sh.Search(attr, &roles_attr));
    EXPECT_TRUE(sh.Search(attr, &roles));
    EXPECT_EQ(roles.size(), roles_attr.size());
    EXPECT_EQ(roles.size(), 4);

    RolePtr role;
    RoleAttributes rattr;
    EXPECT_TRUE(sh.Search(attr, &rattr));
    EXPECT_TRUE(sh.Search(attr, &role));

    EXPECT_TRUE(sh.Search(attr));

    roles_attr.clear();
    roles.clear();

    EXPECT_TRUE(sh.Search(5, &roles_attr));
    EXPECT_TRUE(sh.Search(5, &roles));
    EXPECT_EQ(roles.size(), roles_attr.size());
    EXPECT_EQ(roles.size(), 1);

    EXPECT_TRUE(sh.Search(5, &rattr));
    EXPECT_TRUE(sh.Search(5, &role));
    EXPECT_TRUE(sh.Search(5));

    roles_attr.clear();
    roles.clear();

    attr.set_node_id(i + 10);
    EXPECT_FALSE(sh.Search(attr, &roles_attr));
    EXPECT_FALSE(sh.Search(attr, &roles));
    EXPECT_EQ(roles.size(), roles_attr.size());
    EXPECT_EQ(roles.size(), 0);

    EXPECT_FALSE(sh.Search(attr, &rattr));
    EXPECT_FALSE(sh.Search(attr, &role));

    EXPECT_FALSE(sh.Search(attr));

    roles_attr.clear();
    roles.clear();

    EXPECT_FALSE(sh.Search(i + 10, &roles_attr));
    EXPECT_FALSE(sh.Search(i + 10, &roles));
    EXPECT_EQ(roles.size(), roles_attr.size());
    EXPECT_EQ(roles.size(), 0);

    EXPECT_FALSE(sh.Search(i + 10, &rattr));
    EXPECT_FALSE(sh.Search(i + 10, &role));
    EXPECT_FALSE(sh.Search(i + 10));

    attr.set_node_id(0);

    sh.Remove(attr);
    EXPECT_EQ(sh.Size(), 5);

    attr.set_node_id(i + 10);

    sh.Remove(attr);
    EXPECT_EQ(sh.Size(), 5);

    EXPECT_TRUE(sh.Search(5, &role));
    sh.Remove(i + 10, role);
    EXPECT_EQ(sh.Size(), 5);
    sh.Remove(8, role);
    EXPECT_EQ(sh.Size(), 5);
    sh.Remove(6, role);
    EXPECT_EQ(sh.Size(), 4);
    sh.Remove(5, role);
    EXPECT_EQ(sh.Size(), 3);
    sh.Remove(6);
    EXPECT_EQ(sh.Size(), 3);
    sh.Remove(8);
    EXPECT_EQ(sh.Size(), 2);

    sh.Clear();
    EXPECT_EQ(sh.Size(), 0);
  }
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
