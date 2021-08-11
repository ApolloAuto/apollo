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
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/proto/unit_test.pb.h"

#include "cyber/common/global_data.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"

namespace apollo {
namespace cyber {

TEST(WriterReaderTest, constructor) {
  const std::string channel_name("constructor");
  proto::RoleAttributes attr;
  attr.set_channel_name(channel_name);

  Writer<proto::UnitTest> writer_a(attr);
  EXPECT_FALSE(writer_a.IsInit());
  EXPECT_EQ(writer_a.GetChannelName(), channel_name);

  Reader<proto::UnitTest> reader_a(attr);
  EXPECT_FALSE(reader_a.IsInit());
  EXPECT_EQ(reader_a.GetChannelName(), channel_name);

  attr.set_host_name("caros");
  Writer<proto::UnitTest> writer_b(attr);
  EXPECT_FALSE(writer_b.IsInit());
  EXPECT_EQ(writer_b.GetChannelName(), channel_name);

  Reader<proto::UnitTest> reader_b(attr);
  EXPECT_FALSE(reader_b.IsInit());
  EXPECT_EQ(reader_b.GetChannelName(), channel_name);

  attr.set_process_id(12345);
  Writer<proto::UnitTest> writer_c(attr);
  EXPECT_FALSE(writer_c.IsInit());
  EXPECT_EQ(writer_c.GetChannelName(), channel_name);

  Reader<proto::UnitTest> reader_c(attr);
  EXPECT_FALSE(reader_c.IsInit());
  EXPECT_EQ(reader_c.GetChannelName(), channel_name);
}

TEST(WriterReaderTest, init_and_shutdown) {
  const std::string channel_name_a("init");
  const std::string channel_name_b("shutdown");

  proto::RoleAttributes attr;
  attr.set_channel_name(channel_name_a);
  attr.set_host_name("caros");
  attr.set_process_id(12345);

  Writer<proto::UnitTest> writer_a(attr);
  EXPECT_TRUE(writer_a.Init());
  EXPECT_TRUE(writer_a.IsInit());
  // repeated call
  EXPECT_TRUE(writer_a.Init());

  attr.set_channel_name(channel_name_b);
  attr.set_process_id(54321);
  Reader<proto::UnitTest> reader_a(attr);
  EXPECT_TRUE(reader_a.Init());
  EXPECT_TRUE(reader_a.IsInit());
  // repeated call
  EXPECT_TRUE(reader_a.Init());

  Writer<proto::UnitTest> writer_b(attr);
  EXPECT_TRUE(writer_b.Init());
  EXPECT_TRUE(writer_b.IsInit());

  attr.set_channel_name(channel_name_a);
  attr.set_host_name("sorac");
  attr.set_process_id(12345);
  Reader<proto::UnitTest> reader_b(attr);
  EXPECT_TRUE(reader_b.Init());
  EXPECT_TRUE(reader_b.IsInit());

  Reader<proto::UnitTest> reader_c(attr);
  EXPECT_FALSE(reader_c.Init());
  EXPECT_FALSE(reader_c.IsInit());

  writer_a.Shutdown();
  // repeated call
  writer_a.Shutdown();
  reader_a.Shutdown();
  // repeated call
  reader_a.Shutdown();
  writer_b.Shutdown();
  reader_b.Shutdown();
  reader_c.Shutdown();

  EXPECT_FALSE(writer_a.IsInit());
  EXPECT_FALSE(writer_b.IsInit());
  EXPECT_FALSE(reader_a.IsInit());
  EXPECT_FALSE(reader_b.IsInit());
  EXPECT_FALSE(reader_c.IsInit());
}

TEST(WriterReaderTest, messaging) {
  proto::RoleAttributes attr;
  attr.set_node_name("writer");
  attr.set_channel_name("messaging");
  auto channel_id = common::GlobalData::RegisterChannel(attr.channel_name());
  attr.set_channel_id(channel_id);

  Writer<proto::UnitTest> writer(attr);
  EXPECT_TRUE(writer.Init());

  std::mutex mtx;
  std::vector<proto::UnitTest> recv_msgs;
  attr.set_node_name("reader_a");
  Reader<proto::UnitTest> reader_a(
      attr, [&](const std::shared_ptr<proto::UnitTest>& msg) {
        std::lock_guard<std::mutex> lck(mtx);
        recv_msgs.emplace_back(*msg);
      });
  EXPECT_TRUE(reader_a.Init());

  attr.set_node_name("reader_b");
  Reader<proto::UnitTest> reader_b(
      attr, [&](const std::shared_ptr<proto::UnitTest>& msg) {
        std::lock_guard<std::mutex> lck(mtx);
        recv_msgs.emplace_back(*msg);
      });
  EXPECT_TRUE(reader_b.Init());

  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name("WriterReaderTest");
  msg->set_case_name("messaging");

  writer.Write(msg);
  std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(500));

  EXPECT_EQ(recv_msgs.size(), 2);

  EXPECT_TRUE(writer.HasReader());
  EXPECT_TRUE(reader_a.HasWriter());

  writer.Shutdown();
  reader_a.Shutdown();
  reader_b.Shutdown();
}

TEST(WriterReaderTest, observe) {
  proto::RoleAttributes attr;
  attr.set_node_name("node");
  attr.set_channel_name("channel");
  auto channel_id = common::GlobalData::RegisterChannel(attr.channel_name());
  attr.set_channel_id(channel_id);
  attr.mutable_qos_profile()->set_depth(10);

  Reader<proto::UnitTest> reader(attr);
  reader.Init();
  EXPECT_TRUE(reader.Empty());
  EXPECT_FALSE(reader.HasReceived());
  auto latest = reader.GetLatestObserved();
  auto oldest = reader.GetOldestObserved();
  EXPECT_EQ(latest, nullptr);
  EXPECT_EQ(oldest, nullptr);

  auto msg1 = std::make_shared<proto::UnitTest>();
  msg1->set_case_name("message_1");
  reader.Enqueue(msg1);
  EXPECT_TRUE(reader.HasReceived());

  auto msg2 = std::make_shared<proto::UnitTest>();
  msg2->set_case_name("message_2");
  reader.Enqueue(msg2);

  reader.Observe();
  EXPECT_FALSE(reader.Empty());

  latest = reader.GetLatestObserved();
  oldest = reader.GetOldestObserved();
  ASSERT_NE(latest, nullptr);
  EXPECT_EQ(latest->case_name(), "message_2");
  ASSERT_NE(oldest, nullptr);
  EXPECT_EQ(oldest->case_name(), "message_1");

  auto msg3 = std::make_shared<proto::UnitTest>();
  msg3->set_case_name("message_3");
  for (int i = 0; i < 5; ++i) {
    reader.Enqueue(msg3);
  }
  reader.Observe();
  oldest = reader.GetOldestObserved();
  EXPECT_EQ(oldest->case_name(), "message_1");

  reader.SetHistoryDepth(5);
  reader.Observe();
  oldest = reader.GetOldestObserved();
  EXPECT_EQ(oldest->case_name(), "message_3");

  for (auto it = reader.Begin(); it != reader.End(); ++it) {
    EXPECT_EQ((*it)->case_name(), "message_3");
  }

  reader.ClearData();
  EXPECT_TRUE(reader.Empty());
  EXPECT_FALSE(reader.HasReceived());

  Writer<proto::UnitTest> writer(attr);
  writer.Init();
  writer.Write(msg1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(reader.HasReceived());
  reader.Observe();
  ASSERT_FALSE(reader.Empty());
  latest = reader.GetLatestObserved();
  EXPECT_EQ(latest->case_name(), "message_1");
}

TEST(WriterReaderTest, get_delay_sec) {
  proto::RoleAttributes attr;
  attr.set_node_name("node");
  attr.set_channel_name("channel");
  auto channel_id = common::GlobalData::RegisterChannel(attr.channel_name());
  attr.set_channel_id(channel_id);

  Reader<proto::UnitTest> reader(attr);
  reader.Init();

  EXPECT_LT(reader.GetDelaySec(), 0);
  reader.Enqueue(std::make_shared<proto::UnitTest>());
  EXPECT_GT(reader.GetDelaySec(), 0);
  sleep(1);
  reader.Enqueue(std::make_shared<proto::UnitTest>());
  EXPECT_GT(reader.GetDelaySec(), 1);
}

class Message {
 public:
  uint64_t timestamp;
  std::string content;
  // void GetDescriptorString(const std::string& type, std::string* desc_str) {}
};

TEST(WriterReaderTest, user_defined_message) {
  proto::RoleAttributes attr;
  attr.set_channel_name("/internal_channel");
  auto channel_id = common::GlobalData::RegisterChannel(attr.channel_name());
  attr.set_channel_id(channel_id);
  attr.mutable_qos_profile()->set_depth(10);

  EXPECT_FALSE(message::HasSerializer<Message>::value);
  EXPECT_FALSE(message::HasType<Message>::value);

  auto node = CreateNode("node");
  ASSERT_TRUE(node);

  auto writer = node->CreateWriter<Message>(attr);
  auto reader = node->CreateReader<Message>(attr);

  auto msg = std::make_shared<Message>();
  msg->timestamp = 100;
  msg->content = "message";

  writer->Write(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  writer->Write(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  writer->Write(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  reader->Observe();
  ASSERT_TRUE(reader->HasReceived());
  ASSERT_FALSE(reader->Empty());
  auto latest = reader->GetLatestObserved();
  EXPECT_EQ(latest->timestamp, 100);
  EXPECT_EQ(latest->content, "message");
}

}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  return RUN_ALL_TESTS();
}
