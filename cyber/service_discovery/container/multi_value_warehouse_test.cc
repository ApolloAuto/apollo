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

#include "cyber/service_discovery/container/multi_value_warehouse.h"

#include <memory>
#include <utility>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

using proto::RoleAttributes;

TEST(MultiValueWarehouseTest, test1) {
  { MultiValueWarehouse wh; }

  {
    MultiValueWarehouse wh;
    EXPECT_EQ(wh.Size(), 0);

    RoleAttributes attr;
    attr.set_host_name("caros");
    attr.set_process_id(12345);

    int index = 0;
    {  // 0
      attr.set_node_id(index);
      attr.set_channel_id(index);

      {
        attr.set_id(2 * index);
        auto rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_TRUE(wh.Add(index, rolePtr));

        rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_TRUE(wh.Add(index + 1, rolePtr));

        rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_TRUE(wh.Add(index + 2, rolePtr));

        rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_TRUE(wh.Add(index + 3, rolePtr));
      }
      {
        attr.set_id(2 * index + 1);
        auto rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_FALSE(wh.Add(index, rolePtr, false));
      }
      {
        attr.set_id(2 * index + 2);
        auto rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_TRUE(wh.Add(index, rolePtr));
      }
      {
        attr.set_id(2 * index + 3);
        auto rolePtr = std::make_shared<RoleBase>(attr, 54321);
        EXPECT_TRUE(wh.Add(index++, rolePtr));
      }
    }

    {  // 1
      attr.set_node_id(index);
      attr.set_channel_id(index);
      {
        attr.set_id(2 * index);
        auto rolePtr = std::make_shared<RoleWriter>();
        rolePtr->set_attributes(attr);
        EXPECT_TRUE(wh.Add(index, rolePtr));
        EXPECT_FALSE(wh.Add(index, rolePtr, false));
      }

      {
        attr.set_id(2 * index + 1);
        auto rolePtr = std::make_shared<RoleWriter>();
        rolePtr->set_attributes(attr);
        EXPECT_TRUE(wh.Add(index++, rolePtr));
      }

      EXPECT_EQ(wh.Size(), 8);
    }

    {  // 2
      attr.set_node_id(index);
      attr.set_channel_id(index);
      attr.set_id(2 * index);
      auto rolePtr = std::make_shared<RoleWriter>();
      rolePtr->set_attributes(attr);
      EXPECT_TRUE(wh.Add(index, rolePtr));

      attr.set_id(2 * index + 1);
      auto rolePtr2 = std::make_shared<RoleWriter>();
      rolePtr2->set_attributes(attr);
      EXPECT_TRUE(wh.Add(index++, rolePtr2));

      EXPECT_EQ(wh.Size(), 10);
    }

    {  // 3
      attr.set_node_id(index);
      attr.set_channel_id(index);
      {
        attr.set_id(2 * index);
        auto rolePtr = std::make_shared<RoleWriter>();
        rolePtr->set_attributes(attr);
        EXPECT_TRUE(wh.Add(index, rolePtr));
      }

      {
        attr.set_id(2 * index + 1);
        auto rolePtr = std::make_shared<RoleWriter>();
        rolePtr->set_attributes(attr);
        EXPECT_TRUE(wh.Add(index, rolePtr));
      }

      {
        attr.set_id(2 * index + 2);
        auto rolePtr = std::make_shared<RoleWriter>();
        rolePtr->set_attributes(attr);
        EXPECT_TRUE(wh.Add(index++, rolePtr));
      }

      EXPECT_EQ(wh.Size(), 13);

      EXPECT_TRUE(wh.Search(attr));
      EXPECT_TRUE(wh.Search(3));

      std::vector<RolePtr> roles;
      wh.GetAllRoles(&roles);
      EXPECT_EQ(roles.size(), wh.Size());

      std::vector<RoleAttributes> roles_attr;
      wh.GetAllRoles(&roles_attr);
      EXPECT_EQ(roles.size(), roles_attr.size());

      attr.set_node_id(0);
      attr.set_channel_id(0);
      attr.set_id(0);

      roles.clear();
      EXPECT_TRUE(wh.Search(attr, &roles));
      EXPECT_EQ(roles.size(), 6);

      roles_attr.clear();
      EXPECT_TRUE(wh.Search(attr, &roles_attr));
      EXPECT_EQ(roles_attr.size(), 6);

      attr.set_id(100);
      roles.clear();
      EXPECT_TRUE(wh.Search(attr, &roles));
      EXPECT_EQ(roles.size(), 6);

      roles_attr.clear();
      EXPECT_TRUE(wh.Search(attr, &roles_attr));
      EXPECT_EQ(roles_attr.size(), 6);

      EXPECT_TRUE(wh.Search(attr));
      EXPECT_FALSE(wh.Search(50));

      wh.Remove(0);
      EXPECT_EQ(wh.Size(), 10);
      roles.clear();
      EXPECT_TRUE(wh.Search(attr, &roles));
      EXPECT_EQ(roles.size(), 3);

      roles_attr.clear();
      EXPECT_TRUE(wh.Search(attr, &roles_attr));
      EXPECT_EQ(roles_attr.size(), 3);

      EXPECT_TRUE(wh.Search(attr));

      wh.Remove(attr);
      EXPECT_EQ(wh.Size(), 7);
      roles.clear();
      EXPECT_FALSE(wh.Search(attr, &roles));
      EXPECT_EQ(roles.size(), 0);

      roles_attr.clear();
      EXPECT_FALSE(wh.Search(attr, &roles_attr));
      EXPECT_EQ(roles_attr.size(), 0);

      EXPECT_FALSE(wh.Search(attr));

      roles_attr.clear();
      EXPECT_TRUE(wh.Search(3, &roles_attr));
      EXPECT_EQ(roles_attr.size(), 3);

      roles_attr.clear();
      EXPECT_FALSE(wh.Search(10, &roles_attr));
      EXPECT_EQ(roles_attr.size(), 0);

      roles.clear();
      EXPECT_TRUE(wh.Search(3, &roles));
      EXPECT_EQ(roles.size(), 3);

      roles.clear();
      EXPECT_FALSE(wh.Search(10, &roles));
      EXPECT_EQ(roles.size(), 0);

      RolePtr role;
      EXPECT_TRUE(wh.Search(2, &role));
      EXPECT_TRUE(wh.Search(3, &attr));

      EXPECT_FALSE(wh.Search(10, &role));
      EXPECT_FALSE(wh.Search(10, &attr));

      attr.set_id(0);
      wh.Remove(attr);

      EXPECT_TRUE(wh.Search(1, &role));
      wh.Remove(2, role);

      wh.Remove(100);
      wh.Remove(3);
      wh.Clear();
      EXPECT_EQ(wh.Size(), 0);
    }
  }
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
