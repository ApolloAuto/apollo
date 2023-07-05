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

#include "cyber/service_discovery/role/role.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

using proto::RoleAttributes;

TEST(RoleTest, constructor_getter_setter) {
  RoleBase role_a;
  EXPECT_EQ(role_a.timestamp_ns(), 0);
  role_a.set_timestamp_ns(123);
  EXPECT_EQ(role_a.timestamp_ns(), 123);

  RoleWriter role_writer_a;
  EXPECT_EQ(role_writer_a.timestamp_ns(), 0);

  RoleServer role_server_a;
  EXPECT_EQ(role_server_a.timestamp_ns(), 0);

  RoleAttributes attr;
  attr.set_host_name("caros");
  attr.set_process_id(12345);
  role_a.set_attributes(attr);
  EXPECT_EQ(role_a.attributes().host_name(), "caros");
  EXPECT_EQ(role_a.attributes().process_id(), 12345);

  RoleBase role_b(attr, 321);
  EXPECT_EQ(role_b.timestamp_ns(), 321);

  RoleWriter role_writer_b(attr, 321);
  EXPECT_EQ(role_writer_b.timestamp_ns(), 321);

  RoleWriter role_server_b(attr, 321);
  EXPECT_EQ(role_server_b.timestamp_ns(), 321);
}

TEST(RoleTest, is_earlier_than) {
  RoleBase role_a;
  role_a.set_timestamp_ns(123);
  RoleBase role_b;
  role_b.set_timestamp_ns(456);
  EXPECT_TRUE(role_a.IsEarlierThan(role_b));
  EXPECT_FALSE(role_b.IsEarlierThan(role_a));
}

TEST(RoleTest, rolebase_match) {
  RoleAttributes attr;
  attr.set_host_name("caros");
  attr.set_process_id(12345);
  attr.set_node_id(1);

  RoleBase role(attr, 1234567);

  RoleAttributes target_attr;
  EXPECT_TRUE(role.Match(target_attr));

  target_attr.set_host_name("sorac");
  EXPECT_FALSE(role.Match(target_attr));

  target_attr.set_host_name("caros");
  EXPECT_TRUE(role.Match(target_attr));

  target_attr.set_process_id(54321);
  EXPECT_FALSE(role.Match(target_attr));

  target_attr.set_process_id(12345);
  EXPECT_TRUE(role.Match(target_attr));

  target_attr.set_node_id(2);
  EXPECT_FALSE(role.Match(target_attr));

  target_attr.set_node_id(1);
  EXPECT_TRUE(role.Match(target_attr));
}

TEST(RoleTest, rolewriter_match) {
  RoleAttributes attr;
  attr.set_host_name("caros");
  attr.set_process_id(12345);
  attr.set_node_id(1);

  RoleAttributes target_attr(attr);

  attr.set_channel_id(2);
  attr.set_id(3);

  RoleWriter role(attr, 1234567);

  EXPECT_TRUE(role.Match(target_attr));

  target_attr.set_id(4);
  EXPECT_FALSE(role.Match(target_attr));

  target_attr.set_id(3);
  EXPECT_TRUE(role.Match(target_attr));

  target_attr.set_channel_id(3);
  EXPECT_FALSE(role.Match(target_attr));

  target_attr.set_channel_id(2);
  EXPECT_TRUE(role.Match(target_attr));
}

TEST(RoleTest, roleserver_match) {
  RoleAttributes attr;
  attr.set_host_name("caros");
  attr.set_process_id(12345);
  attr.set_node_id(1);

  RoleAttributes target_attr(attr);

  attr.set_service_id(2);

  RoleServer role(attr, 1234567);

  EXPECT_TRUE(role.Match(target_attr));

  target_attr.set_service_id(3);
  EXPECT_FALSE(role.Match(target_attr));

  target_attr.set_service_id(2);
  EXPECT_TRUE(role.Match(target_attr));
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
