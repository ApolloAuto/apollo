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
#include "cybertron/transport/lower_reach/shm_lower_reach.h"
#include "cybertron/transport/upper_reach/shm_upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

class ShmReachTest : public ::testing::Test {
 protected:
  using Upper = ShmUpperReach<proto::UnitTest>;
  using Lower = ShmLowerReach<proto::UnitTest>;
  using UpperPtr = std::shared_ptr<UpperReach<proto::UnitTest>>;
  using LowerPtr = std::shared_ptr<LowerReach<proto::UnitTest>>;

  ShmReachTest() : channel_name_("shm_channel") {}

  virtual ~ShmReachTest() {}

  virtual void SetUp() {
    RoleAttributes attr;
    attr.set_host_name(common::GlobalData::Instance()->HostName());
    attr.set_channel_name(channel_name_);
    attr.set_channel_id(common::Hash(channel_name_));
    upper_reach_a_ = std::make_shared<Upper>(attr);
    upper_reach_b_ = std::make_shared<Upper>(attr);

    upper_reach_a_->Enable();
    upper_reach_b_->Enable();
  }

  virtual void TearDown() {
    upper_reach_a_ = nullptr;
    upper_reach_b_ = nullptr;
  }

  std::string channel_name_;
  UpperPtr upper_reach_a_ = nullptr;
  UpperPtr upper_reach_b_ = nullptr;
};

TEST_F(ShmReachTest, constructor) {
  RoleAttributes attr;
  UpperPtr upper = std::make_shared<Upper>(attr);
  LowerPtr lower = std::make_shared<Lower>(attr, nullptr);

  EXPECT_EQ(upper->seq_num(), 0);

  auto& upper_id = upper->id();
  auto& lower_id = lower->id();

  EXPECT_NE(upper_id.ToString(), lower_id.ToString());
}

TEST_F(ShmReachTest, enable_and_disable) {
  // repeated call
  upper_reach_a_->Enable();

  std::vector<proto::UnitTest> msgs;
  RoleAttributes attr;
  attr.set_channel_name(channel_name_);
  attr.set_channel_id(common::Hash(channel_name_));
  LowerPtr lower = std::make_shared<Lower>(
      attr, [&msgs](const std::shared_ptr<proto::UnitTest>& msg,
                    const MessageInfo& msg_info, const RoleAttributes& attr) {
        (void)msg_info;
        (void)attr;
        msgs.emplace_back(*msg);
      });

  lower->Enable();
  // repeated call
  lower->Enable();

  LowerPtr lower_null_cb = std::make_shared<Lower>(attr, nullptr);
  lower_null_cb->Enable();

  auto msg = std::make_shared<proto::UnitTest>();
  msg->set_class_name("ShmReachTest");
  msg->set_case_name("enable_and_disable");

  EXPECT_TRUE(upper_reach_a_->Transmit(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(msgs.size(), 1);

  EXPECT_TRUE(upper_reach_b_->Transmit(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(msgs.size(), 2);

  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), "ShmReachTest");
    EXPECT_EQ(item.case_name(), "enable_and_disable");
  }

  upper_reach_b_->Disable(lower->attributes());
  EXPECT_FALSE(upper_reach_b_->Transmit(msg));

  upper_reach_b_->Enable(lower->attributes());
  auto& upper_b_attr = upper_reach_b_->attributes();

  lower->Disable();
  lower->Enable(upper_b_attr);

  msgs.clear();
  EXPECT_TRUE(upper_reach_a_->Transmit(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(msgs.size(), 0);

  EXPECT_TRUE(upper_reach_b_->Transmit(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(msgs.size(), 1);
  for (auto& item : msgs) {
    EXPECT_EQ(item.class_name(), "ShmReachTest");
    EXPECT_EQ(item.case_name(), "enable_and_disable");
  }

  lower->Disable(upper_b_attr);
  msgs.clear();
  EXPECT_TRUE(upper_reach_b_->Transmit(msg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(msgs.size(), 0);
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
