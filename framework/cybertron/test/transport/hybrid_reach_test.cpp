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

#include <thread>

#include "cybertron/common/global_data.h"
#include "cybertron/common/util.h"
#include "cybertron/proto/unit_test.pb.h"
#include "cybertron/transport/lower_reach/hybrid_lower_reach.h"
#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/transport/transport.h"
#include "cybertron/transport/upper_reach/hybrid_upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

class HybridReachTest : public ::testing::Test {
 protected:
  using Upper = HybridUpperReach<proto::UnitTest>;
  using Lower = HybridLowerReach<proto::UnitTest>;
  using UpperPtr = std::shared_ptr<UpperReach<proto::UnitTest>>;
  using LowerPtr = std::shared_ptr<LowerReach<proto::UnitTest>>;

  HybridReachTest() : channel_name_("hybrid_channel") {}

  virtual ~HybridReachTest() {}

  virtual void SetUp() {
    RoleAttributes attr;
    attr.set_host_name(common::GlobalData::Instance()->HostName());
    attr.set_process_id(common::GlobalData::Instance()->ProcessId());
    attr.set_channel_name(channel_name_);
    attr.set_channel_id(common::Hash(channel_name_));
    attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
    upper_reach_a_ = std::make_shared<Upper>(attr, Transport::participant());

    attr.set_process_id(54321);
    attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_TOPO_CHANGE);
    upper_reach_b_ = std::make_shared<Upper>(attr, Transport::participant());
  }

  virtual void TearDown() {
    upper_reach_a_ = nullptr;
    upper_reach_b_ = nullptr;
  }

  std::string channel_name_;
  UpperPtr upper_reach_a_ = nullptr;
  UpperPtr upper_reach_b_ = nullptr;
};

TEST_F(HybridReachTest, constructor) {
  RoleAttributes attr;
  UpperPtr upper = std::make_shared<Upper>(attr, Transport::participant());
  LowerPtr lower =
      std::make_shared<Lower>(attr, nullptr, Transport::participant());

  EXPECT_EQ(upper->seq_num(), 0);

  auto& upper_id = upper->id();
  auto& lower_id = lower->id();

  EXPECT_NE(upper_id.ToString(), lower_id.ToString());
}

TEST_F(HybridReachTest, enable_and_disable_no_param) {
  upper_reach_a_->Enable();
  upper_reach_b_->Enable();

  RoleAttributes attr;
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::vector<proto::UnitTest> msgs;
  LowerPtr lower = std::make_shared<Lower>(
      attr,
      [&msgs](const std::shared_ptr<proto::UnitTest>& msg,
              const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  lower->Enable();

  auto msg = std::make_shared<proto::UnitTest>();
  std::string class_name("HybridReachTest");
  std::string case_name("enable_and_disable_no_param");
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  upper_reach_a_->Transmit(msg);
  upper_reach_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 6);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  upper_reach_b_->Disable();
  upper_reach_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 0);

  msgs.clear();
  lower->Disable();
  upper_reach_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 0);
}

TEST_F(HybridReachTest, enable_and_disable_with_param_no_relation) {
  RoleAttributes attr;
  attr.set_host_name(common::GlobalData::Instance()->HostName());
  attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name("enable_and_disable_with_param_no_relation");
  attr.set_channel_id(
      common::Hash("enable_and_disable_with_param_no_relation"));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  LowerPtr lower_a = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_TOPO_CHANGE);
  LowerPtr lower_b = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name("HybridReachTest");
  msg->set_case_name("enable_and_disable_with_param_no_relation");

  upper_reach_a_->Enable(lower_a->attributes());
  upper_reach_a_->Enable(lower_b->attributes());
  lower_a->Enable(upper_reach_a_->attributes());
  lower_b->Enable(upper_reach_a_->attributes());

  upper_reach_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 0);

  msgs.clear();
  upper_reach_a_->Disable(lower_a->attributes());
  upper_reach_a_->Disable(lower_b->attributes());
  lower_a->Disable(upper_reach_a_->attributes());
  lower_b->Disable(upper_reach_a_->attributes());
}

TEST_F(HybridReachTest, enable_and_disable_with_param_same_process) {
  RoleAttributes attr;
  attr.set_host_name(common::GlobalData::Instance()->HostName());
  attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  LowerPtr lower_a = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_TOPO_CHANGE);
  LowerPtr lower_b = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  std::string class_name("HybridReachTest");
  std::string case_name("enable_and_disable_with_param_same_process");
  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  // this msg will lose
  upper_reach_a_->Transmit(msg);

  upper_reach_a_->Enable(lower_a->attributes());
  upper_reach_a_->Enable(lower_b->attributes());
  lower_a->Enable(upper_reach_a_->attributes());
  lower_b->Enable(upper_reach_a_->attributes());
  // repeated call
  lower_b->Enable(upper_reach_a_->attributes());

  upper_reach_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(msgs.size(), 2);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  upper_reach_a_->Disable(lower_a->attributes());
  upper_reach_a_->Disable(lower_b->attributes());
  lower_a->Disable(upper_reach_a_->attributes());
  lower_b->Disable(upper_reach_a_->attributes());
  // repeated call
  lower_b->Disable(upper_reach_a_->attributes());

  upper_reach_a_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(msgs.size(), 0);
}

TEST_F(HybridReachTest, enable_and_disable_with_param_same_host_diff_proc) {
  RoleAttributes attr;
  attr.set_host_name(common::GlobalData::Instance()->HostName());
  attr.set_process_id(1);
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  LowerPtr lower_a = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_TOPO_CHANGE);
  LowerPtr lower_b = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  std::string class_name("HybridReachTest");
  std::string case_name("enable_and_disable_with_param_same_host_diff_proc");
  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  upper_reach_b_->Transmit(msg);

  upper_reach_b_->Enable(lower_a->attributes());
  upper_reach_b_->Enable(lower_b->attributes());
  lower_a->Enable(upper_reach_b_->attributes());
  lower_b->Enable(upper_reach_b_->attributes());

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  upper_reach_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // 2 from lower_b(transient_local), 1 from lower_a(volatile)
  EXPECT_EQ(msgs.size(), 3);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  upper_reach_b_->Disable(lower_a->attributes());
  upper_reach_b_->Disable(lower_b->attributes());
  lower_a->Disable(upper_reach_b_->attributes());
  lower_b->Disable(upper_reach_b_->attributes());

  upper_reach_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(msgs.size(), 0);
}

TEST_F(HybridReachTest, enable_and_disable_with_param_diff_host) {
  RoleAttributes attr;
  attr.set_host_name("sorac");
  attr.set_process_id(12345);
  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));

  std::mutex mtx;
  std::vector<proto::UnitTest> msgs;
  LowerPtr lower_a = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_TOPO_CHANGE);
  LowerPtr lower_b = std::make_shared<Lower>(
      attr,
      [&](const std::shared_ptr<proto::UnitTest>& msg,
          const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        std::lock_guard<std::mutex> lock(mtx);
        msgs.emplace_back(*msg);
      },
      Transport::participant());

  std::string class_name("HybridReachTest");
  std::string case_name("enable_and_disable_with_param_same_host_diff_proc");
  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name(class_name);
  msg->set_case_name(case_name);

  upper_reach_b_->Enable(lower_a->attributes());
  upper_reach_b_->Enable(lower_b->attributes());
  lower_a->Enable(upper_reach_b_->attributes());
  lower_b->Enable(upper_reach_b_->attributes());

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  upper_reach_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // 1 from lower_b(transient_local), 1 from lower_a(volatile)
  EXPECT_EQ(msgs.size(), 2);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), class_name);
    EXPECT_EQ(item.case_name(), case_name);
  }

  msgs.clear();
  upper_reach_b_->Disable(lower_a->attributes());
  upper_reach_b_->Disable(lower_b->attributes());
  upper_reach_b_->Transmit(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(msgs.size(), 0);
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
