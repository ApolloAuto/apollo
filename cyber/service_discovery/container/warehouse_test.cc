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

#include "cyber/service_discovery/container/multi_value_warehouse.h"
#include "cyber/service_discovery/container/single_value_warehouse.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

using proto::RoleAttributes;

class WarehouseTest : public ::testing::Test {
 protected:
  WarehouseTest() : key_num_(256) {}

  virtual ~WarehouseTest() {}

  virtual void SetUp() {
    FillSingle();
    FillMulti();
  }

  virtual void TearDown() {
    single_.Clear();
    multi_.Clear();
  }

  int key_num_;
  SingleValueWarehouse single_;
  MultiValueWarehouse multi_;

 private:
  void FillSingle() {
    RoleAttributes attr;
    attr.set_host_name("caros");
    attr.set_process_id(12345);
    for (int i = 0; i < key_num_; ++i) {
      auto role = std::make_shared<RoleBase>();
      attr.set_node_id(i);
      role->set_attributes(attr);
      single_.Add(i, role);
    }
  }

  void FillMulti() {
    RoleAttributes attr;
    attr.set_host_name("caros");
    attr.set_process_id(12345);
    for (int i = 0; i < key_num_; ++i) {
      auto role_a = std::make_shared<RoleWriter>();
      attr.set_node_id(i);
      attr.set_channel_id(i);
      attr.set_id(2 * i);
      role_a->set_attributes(attr);
      multi_.Add(i, role_a);
      attr.set_id(2 * i + 1);
      auto role_b = std::make_shared<RoleWriter>();
      role_b->set_attributes(attr);
      multi_.Add(i, role_b);
    }
  }
};

TEST_F(WarehouseTest, size) {
  EXPECT_EQ(single_.Size(), key_num_);
  EXPECT_EQ(multi_.Size(), 2 * key_num_);

  single_.Clear();
  multi_.Clear();

  EXPECT_EQ(single_.Size(), 0);
  EXPECT_EQ(multi_.Size(), 0);
}

TEST_F(WarehouseTest, add) {
  auto role = std::make_shared<RoleBase>();
  EXPECT_TRUE(single_.Add(key_num_, role, false));
  EXPECT_FALSE(single_.Add(key_num_, role, false));

  EXPECT_TRUE(multi_.Add(key_num_, role, false));
  EXPECT_FALSE(multi_.Add(key_num_, role, false));
}

TEST_F(WarehouseTest, remove) {
  // key
  single_.Remove(0);
  EXPECT_EQ(single_.Size(), key_num_ - 1);

  multi_.Remove(0);
  EXPECT_EQ(multi_.Size(), 2 * key_num_ - 2);

  // key and role
  RoleAttributes attr;
  attr.set_host_name("caros");
  attr.set_process_id(12345);
  attr.set_node_id(0);
  auto role = std::make_shared<RoleBase>();
  role->set_attributes(attr);
  single_.Remove(0, role);
  single_.Remove(1, role);
  EXPECT_EQ(single_.Size(), key_num_ - 1);
  attr.set_node_id(1);
  role->set_attributes(attr);
  single_.Remove(1, role);
  EXPECT_EQ(single_.Size(), key_num_ - 2);

  attr.set_channel_id(1);
  attr.set_id(2);
  role = std::make_shared<RoleWriter>();
  role->set_attributes(attr);
  multi_.Remove(0, role);
  EXPECT_EQ(multi_.Size(), 2 * key_num_ - 2);
  multi_.Remove(1, role);
  EXPECT_EQ(multi_.Size(), 2 * key_num_ - 3);
  attr.set_id(5);
  role->set_attributes(attr);
  multi_.Remove(1, role);
  EXPECT_EQ(multi_.Size(), 2 * key_num_ - 3);

  // attr
  RoleAttributes target_attr;
  target_attr.set_host_name("caros");
  target_attr.set_process_id(54321);
  single_.Remove(target_attr);
  EXPECT_EQ(single_.Size(), key_num_ - 2);
  multi_.Remove(target_attr);
  EXPECT_EQ(multi_.Size(), 2 * key_num_ - 3);

  target_attr.set_process_id(12345);
  single_.Remove(target_attr);
  EXPECT_EQ(single_.Size(), 0);
  multi_.Remove(target_attr);
  EXPECT_EQ(multi_.Size(), 0);
}

TEST_F(WarehouseTest, search) {
  // key
  for (int i = 0; i < key_num_; ++i) {
    EXPECT_TRUE(single_.Search(i));
    EXPECT_TRUE(multi_.Search(i));
  }

  // key and role
  RolePtr role;
  for (int i = 0; i < key_num_; ++i) {
    EXPECT_TRUE(single_.Search(i, &role));
    EXPECT_TRUE(multi_.Search(i, &role));
  }

  // key and role attr
  RoleAttributes attr;
  for (int i = 0; i < key_num_; ++i) {
    EXPECT_TRUE(single_.Search(i, &attr));
    EXPECT_EQ(attr.node_id(), i);

    EXPECT_TRUE(multi_.Search(i, &attr));
    EXPECT_EQ(attr.channel_id(), i);
  }

  // key and role vec
  std::vector<RolePtr> role_vec;
  for (int i = 0; i < key_num_; ++i) {
    EXPECT_TRUE(single_.Search(i, &role_vec));
    EXPECT_EQ(role_vec.size(), 1);
    role_vec.clear();

    EXPECT_TRUE(multi_.Search(i, &role_vec));
    EXPECT_EQ(role_vec.size(), 2);
    role_vec.clear();
  }

  // key and role attr vec
  std::vector<RoleAttributes> role_attr_vec;
  for (int i = 0; i < key_num_; ++i) {
    EXPECT_TRUE(single_.Search(i, &role_attr_vec));
    EXPECT_EQ(role_attr_vec.size(), 1);
    role_attr_vec.clear();

    EXPECT_TRUE(multi_.Search(i, &role_attr_vec));
    EXPECT_EQ(role_attr_vec.size(), 2);
    role_attr_vec.clear();
  }

  // attr
  RoleAttributes target_attr;
  for (int i = 0; i < key_num_; ++i) {
    target_attr.set_node_id(i);
    EXPECT_TRUE(single_.Search(target_attr));

    target_attr.set_channel_id(i);
    EXPECT_TRUE(multi_.Search(target_attr));
  }

  // attr and role
  for (int i = 0; i < key_num_; ++i) {
    target_attr.set_node_id(i);
    EXPECT_TRUE(single_.Search(target_attr, &role));

    target_attr.set_channel_id(i);
    EXPECT_TRUE(multi_.Search(target_attr, &role));
  }

  // attr and role attr
  for (int i = 0; i < key_num_; ++i) {
    target_attr.set_node_id(i);
    EXPECT_TRUE(single_.Search(target_attr, &attr));
    EXPECT_EQ(attr.node_id(), i);

    target_attr.set_channel_id(i);
    EXPECT_TRUE(multi_.Search(target_attr, &attr));
    EXPECT_EQ(attr.channel_id(), i);
  }

  // attr and role vec
  for (int i = 0; i < key_num_; ++i) {
    target_attr.set_node_id(i);
    EXPECT_TRUE(single_.Search(target_attr, &role_vec));
    EXPECT_EQ(role_vec.size(), 1);
    role_vec.clear();

    target_attr.set_channel_id(i);
    EXPECT_TRUE(multi_.Search(target_attr, &role_vec));
    EXPECT_EQ(role_vec.size(), 2);
    role_vec.clear();
  }

  // attr and role attr vec
  for (int i = 0; i < key_num_; ++i) {
    target_attr.set_node_id(i);
    EXPECT_TRUE(single_.Search(target_attr, &role_attr_vec));
    EXPECT_EQ(role_attr_vec.size(), 1);
    role_attr_vec.clear();

    target_attr.set_channel_id(i);
    EXPECT_TRUE(multi_.Search(target_attr, &role_attr_vec));
    EXPECT_EQ(role_attr_vec.size(), 2);
    role_attr_vec.clear();
  }
}

TEST_F(WarehouseTest, get_all_roles) {
  // role vec
  std::vector<RolePtr> role_vec;
  single_.GetAllRoles(&role_vec);
  EXPECT_EQ(role_vec.size(), key_num_);
  role_vec.clear();

  multi_.GetAllRoles(&role_vec);
  EXPECT_EQ(role_vec.size(), 2 * key_num_);

  // role attr vec
  std::vector<RoleAttributes> role_attr_vec;
  single_.GetAllRoles(&role_attr_vec);
  EXPECT_EQ(role_attr_vec.size(), key_num_);
  role_attr_vec.clear();

  multi_.GetAllRoles(&role_attr_vec);
  EXPECT_EQ(role_attr_vec.size(), 2 * key_num_);
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
