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

#include "cyber/common/global_data.h"
#include "cyber/common/util.h"
#include "cyber/init.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/qos/qos_profile_conf.h"
#include "cyber/transport/receiver/hybrid_receiver.h"
#include "cyber/transport/transmitter/hybrid_transmitter.h"
#include "cyber/transport/transport.h"

namespace apollo {
namespace cyber {
namespace transport {

class HybridTransceiverTest : public ::testing::Test {
 protected:
  using TransmitterPtr = std::shared_ptr<Transmitter<proto::UnitTest>>;
  using ReceiverPtr = std::shared_ptr<Receiver<proto::UnitTest>>;

  HybridTransceiverTest() : channel_name_("hybrid_channel") {}

  virtual ~HybridTransceiverTest() {}

  virtual void SetUp() {
    RoleAttributes attr;
    attr.set_host_name(common::GlobalData::Instance()->HostName());
    attr.set_host_ip(common::GlobalData::Instance()->HostIp());
    attr.set_process_id(common::GlobalData::Instance()->ProcessId());
    attr.set_channel_name(channel_name_);
    attr.set_channel_id(common::Hash(channel_name_));
    attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
    transmitter_a_ = std::make_shared<HybridTransmitter<proto::UnitTest>>(
        attr, Transport::Instance()->participant());

    attr.set_process_id(common::GlobalData::Instance()->ProcessId() + 1);
    attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
    transmitter_b_ = std::make_shared<HybridTransmitter<proto::UnitTest>>(
        attr, Transport::Instance()->participant());
  }

  virtual void TearDown() {
    transmitter_a_ = nullptr;
    transmitter_b_ = nullptr;
  }

  std::string channel_name_;
  TransmitterPtr transmitter_a_ = nullptr;
  TransmitterPtr transmitter_b_ = nullptr;
};

TEST_F(HybridTransceiverTest, constructor) {
  RoleAttributes attr;
  TransmitterPtr transmitter =
      std::make_shared<HybridTransmitter<proto::UnitTest>>(
          attr, Transport::Instance()->participant());
  ReceiverPtr receiver = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr, nullptr, Transport::Instance()->participant());

  EXPECT_EQ(transmitter->seq_num(), 0);

  auto& transmitter_id = transmitter->id();
  auto& receiver_id = receiver->id();

  EXPECT_NE(transmitter_id.ToString(), receiver_id.ToString());
}

TEST_F(HybridTransceiverTest, enable_and_disable_with_param_no_relation) {
  RoleAttributes attr;
  attr.set_host_name(common::GlobalData::Instance()->HostName());
  attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name("enable_and_disable_with_param_no_relation");
  attr.set_channel_id(
      common::Hash("enable_and_disable_with_param_no_relation"));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  ReceiverPtr receiver_a = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  ReceiverPtr receiver_b = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name("HybridTransceiverTest");
  msg->set_case_name("enable_and_disable_with_param_no_relation");

  transmitter_a_->Enable(receiver_a->attributes());
  transmitter_a_->Enable(receiver_b->attributes());
  receiver_a->Enable(transmitter_a_->attributes());
  receiver_b->Enable(transmitter_a_->attributes());

  transmitter_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 0);

  msgs.clear();
  transmitter_a_->Disable(receiver_a->attributes());
  transmitter_a_->Disable(receiver_b->attributes());
  receiver_a->Disable(transmitter_a_->attributes());
  receiver_b->Disable(transmitter_a_->attributes());
}

TEST_F(HybridTransceiverTest, enable_and_disable_with_param_same_process) {
  RoleAttributes attr;
  attr.set_host_name(common::GlobalData::Instance()->HostName());
  attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  ReceiverPtr receiver_a = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  ReceiverPtr receiver_b = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  std::string class_name("HybridTransceiverTest");
  std::string case_name("enable_and_disable_with_param_same_process");
  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  // this msg will lose
  transmitter_a_->Transmit(msg);

  transmitter_a_->Enable(receiver_a->attributes());
  transmitter_a_->Enable(receiver_b->attributes());
  receiver_a->Enable(transmitter_a_->attributes());
  receiver_b->Enable(transmitter_a_->attributes());
  // repeated call
  receiver_b->Enable(transmitter_a_->attributes());

  transmitter_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 2);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  transmitter_a_->Disable(receiver_a->attributes());
  transmitter_a_->Disable(receiver_b->attributes());
  receiver_a->Disable(transmitter_a_->attributes());
  receiver_b->Disable(transmitter_a_->attributes());
  // repeated call
  receiver_b->Disable(transmitter_a_->attributes());

  transmitter_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(msgs.size(), 0);
}

TEST_F(HybridTransceiverTest,
       enable_and_disable_with_param_same_host_diff_proc) {
  RoleAttributes attr;
  attr.set_host_name(common::GlobalData::Instance()->HostName());
  attr.set_process_id(1);
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  ReceiverPtr receiver_a = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  ReceiverPtr receiver_b = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  std::string class_name("HybridTransceiverTest");
  std::string case_name("enable_and_disable_with_param_same_host_diff_proc");
  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  transmitter_b_->Transmit(msg);

  transmitter_b_->Enable(receiver_a->attributes());
  transmitter_b_->Enable(receiver_b->attributes());
  receiver_a->Enable(transmitter_b_->attributes());
  receiver_b->Enable(transmitter_b_->attributes());

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  transmitter_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // 1 from receiver_b, 1 from receiver_a
  EXPECT_EQ(msgs.size(), 2);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  transmitter_b_->Disable(receiver_a->attributes());
  transmitter_b_->Disable(receiver_b->attributes());
  receiver_a->Disable(transmitter_b_->attributes());
  receiver_b->Disable(transmitter_b_->attributes());

  transmitter_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(msgs.size(), 0);
}

TEST_F(HybridTransceiverTest, enable_and_disable_with_param_diff_host) {
  RoleAttributes attr;
  attr.set_host_name("sorac");
  attr.set_process_id(12345);
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  ReceiverPtr receiver_a = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  ReceiverPtr receiver_b = std::make_shared<HybridReceiver<proto::UnitTest>>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::Instance()->participant());

  std::string class_name("HybridTransceiverTest");
  std::string case_name("enable_and_disable_with_param_same_host_diff_proc");
  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  transmitter_b_->Enable(receiver_a->attributes());
  transmitter_b_->Enable(receiver_b->attributes());
  receiver_a->Enable(transmitter_b_->attributes());
  receiver_b->Enable(transmitter_b_->attributes());

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  transmitter_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // 1 from receiver_b, 1 from receiver_a
  EXPECT_EQ(msgs.size(), 2);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  transmitter_b_->Disable(receiver_a->attributes());
  transmitter_b_->Disable(receiver_b->attributes());
  transmitter_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(msgs.size(), 0);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  apollo::cyber::transport::Transport::Instance();
  auto res = RUN_ALL_TESTS();
  apollo::cyber::transport::Transport::Instance()->Shutdown();
  return res;
}
