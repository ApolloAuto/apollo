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

#include "cybertron/init.h"
#include "cybertron/node/node.h"
#include "cybertron/proto/unit_test.pb.h"

namespace apollo {
namespace cybertron {

TEST(NodeTest, constructor) {
  Node node("constructor");
  EXPECT_EQ(node.Name(), "constructor");
  EXPECT_EQ(node.node_channel_impl_->NodeName(), "constructor");
}

TEST(NodeTest, create_writer_with_channel_name) {
  Node node("create_writer_with_channel_name");

  auto writer = node.CreateWriter<proto::UnitTest>("channel");
  ASSERT_TRUE(writer != nullptr);
  EXPECT_EQ(writer->GetChannelName(), "channel");
  EXPECT_TRUE(writer->inited());
  writer->Shutdown();
}

TEST(NodeTest, create_writer_with_attr) {
  Node node("create_writer_with_attr");

  proto::RoleAttributes attr;
  attr.set_channel_name("channel");
  auto writer_a = node.CreateWriter<proto::UnitTest>(attr);
  ASSERT_TRUE(writer_a != nullptr);
  EXPECT_EQ(writer_a->GetChannelName(), "channel");
  EXPECT_TRUE(writer_a->inited());
  writer_a->Shutdown();

  attr.set_message_type("apollo.cybertron.proto.UnitTest");
  auto writer_b = node.CreateWriter<proto::UnitTest>(attr);
  ASSERT_TRUE(writer_b != nullptr);
  EXPECT_EQ(writer_b->GetChannelName(), "channel");
  EXPECT_TRUE(writer_b->inited());
  writer_b->Shutdown();

  attr.set_proto_desc("");
  auto writer_c = node.CreateWriter<proto::UnitTest>(attr);
  ASSERT_TRUE(writer_c != nullptr);
  EXPECT_EQ(writer_c->GetChannelName(), "channel");
  EXPECT_TRUE(writer_c->inited());
  writer_c->Shutdown();
}

TEST(NodeTest, create_reader_with_channel_name) {
  Node node("create_reader_with_channel_name");

  auto reader = node.CreateReader<proto::UnitTest>("channel", nullptr);
  ASSERT_TRUE(reader != nullptr);
  EXPECT_EQ(reader->GetChannelName(), "channel");
  EXPECT_TRUE(reader->inited());
  reader->Shutdown();
}

TEST(NodeTest, create_reader_with_attr) {
  Node node("create_reader_with_attr");

  proto::RoleAttributes attr;
  attr.set_channel_name("channel");
  auto reader_a = node.CreateReader<proto::UnitTest>(attr, nullptr);
  ASSERT_TRUE(reader_a != nullptr);
  EXPECT_EQ(reader_a->GetChannelName(), "channel");
  EXPECT_TRUE(reader_a->inited());
  reader_a->Shutdown();

  auto reader_b = node.CreateReader<proto::UnitTest>(attr);
  ASSERT_TRUE(reader_b != nullptr);
  EXPECT_EQ(reader_b->GetChannelName(), "channel");
  EXPECT_TRUE(reader_b->inited());
  reader_b->Shutdown();
}

TEST(NodeTest, create_service) {
  Node node("create_service");
  auto service = node.CreateService<proto::UnitTest, proto::UnitTest>(
      "create_service", [](const std::shared_ptr<proto::UnitTest>&,
                           std::shared_ptr<proto::UnitTest>&) {});
  ASSERT_TRUE(service != nullptr);
}

TEST(NodeTest, create_client) {
  Node node("create_client");
  auto client =
      node.CreateClient<proto::UnitTest, proto::UnitTest>("create_client");
  ASSERT_TRUE(client != nullptr);
}

}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
