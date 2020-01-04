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

#include "cyber/node/reader.h"

#include <string>
#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace reader {

using proto::Chatter;

auto callback = [](const std::shared_ptr<proto::Chatter>& msg) {
  AINFO << "msg size = " << msg->ByteSize();
};

TEST(ReaderTest, test1) {
  proto::RoleAttributes role;
  {
    Reader<Chatter> r(role, callback, 0);
    EXPECT_EQ(r.PendingQueueSize(), 0);
    EXPECT_TRUE(r.GetChannelName().empty());
    EXPECT_TRUE(r.Init());
    EXPECT_TRUE(r.GetChannelName().empty());
    EXPECT_TRUE(r.IsInit());

    r.Observe();

    EXPECT_FALSE(r.HasReceived());
    EXPECT_TRUE(r.Empty());
    EXPECT_LT(r.GetDelaySec(), 0);

    r.ClearData();
    r.Shutdown();
  }

  {
    Reader<Chatter> r(role, callback, 100);
    EXPECT_EQ(r.PendingQueueSize(), 100);
    EXPECT_TRUE(r.GetChannelName().empty());
    EXPECT_TRUE(r.Init());
    EXPECT_TRUE(r.GetChannelName().empty());
    EXPECT_TRUE(r.IsInit());

    r.Observe();

    EXPECT_FALSE(r.HasReceived());
    EXPECT_TRUE(r.Empty());
    EXPECT_LT(r.GetDelaySec(), 0);

    r.ClearData();
    r.Shutdown();
  }

  auto qos = role.mutable_qos_profile();
  qos->set_history(proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_depth(1);
  qos->set_mps(0);
  qos->set_reliability(proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(proto::QosDurabilityPolicy::DURABILITY_VOLATILE);
  role.set_channel_name("/chatter0");

  {
    Reader<Chatter> r(role, nullptr, 100);
    EXPECT_EQ(r.PendingQueueSize(), 100);
    EXPECT_EQ(r.GetChannelName(), "/chatter0");
    EXPECT_TRUE(r.Init());
    EXPECT_EQ(r.GetChannelName(), "/chatter0");
    EXPECT_TRUE(r.IsInit());

    r.Observe();

    EXPECT_FALSE(r.HasReceived());
    EXPECT_TRUE(r.Empty());
    EXPECT_LT(r.GetDelaySec(), 0);

    r.ClearData();
    r.Shutdown();
  }

  {
    Reader<Chatter> r(role, callback, 100);
    EXPECT_EQ(r.PendingQueueSize(), 100);
    EXPECT_EQ(r.GetChannelName(), "/chatter0");
    EXPECT_TRUE(r.Init());
    EXPECT_EQ(r.GetChannelName(), "/chatter0");
    EXPECT_TRUE(r.IsInit());

    r.Observe();

    EXPECT_FALSE(r.HasReceived());
    EXPECT_TRUE(r.Empty());
    EXPECT_LT(r.GetDelaySec(), 0);

    r.ClearData();
    r.Shutdown();
  }
}

TEST(ReaderTest, test2) {
  proto::RoleAttributes role;
  auto qos = role.mutable_qos_profile();
  qos->set_history(proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_depth(0);
  qos->set_mps(0);
  qos->set_reliability(proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(proto::QosDurabilityPolicy::DURABILITY_VOLATILE);
  role.set_channel_name("/chatter0");

  Reader<Chatter> r(role, callback, 100);
  EXPECT_EQ(r.PendingQueueSize(), 100);
  EXPECT_EQ(r.GetChannelName(), "/chatter0");

  EXPECT_TRUE(r.Init());
  r.SetHistoryDepth(1);
  EXPECT_TRUE(r.Init());
  {
    auto c = std::make_shared<Chatter>();
    c->set_timestamp(Time::Now().ToNanosecond());
    c->set_lidar_timestamp(Time::Now().ToNanosecond());
    c->set_seq(1);
    c->set_content("ChatterMsg");
    r.Enqueue(c);
  }
  EXPECT_EQ(r.GetHistoryDepth(), 1);
  EXPECT_GT(r.GetDelaySec(), 0);

  r.SetHistoryDepth(0);
  {
    auto c = std::make_shared<Chatter>();
    c->set_timestamp(Time::Now().ToNanosecond());
    c->set_lidar_timestamp(Time::Now().ToNanosecond());
    c->set_seq(2);
    c->set_content("ChatterMsg");
    r.Enqueue(c);
  }
  EXPECT_EQ(r.GetHistoryDepth(), 0);
  EXPECT_EQ(r.Begin(), r.End());
  EXPECT_GT(r.GetDelaySec(), 0);

  r.SetHistoryDepth(3);
  {
    auto c = std::make_shared<Chatter>();
    c->set_timestamp(Time::Now().ToNanosecond());
    c->set_lidar_timestamp(Time::Now().ToNanosecond());
    c->set_seq(3);
    c->set_content("ChatterMsg");
    r.Enqueue(c);
  }
  {
    auto c = std::make_shared<Chatter>();
    c->set_timestamp(Time::Now().ToNanosecond());
    c->set_lidar_timestamp(Time::Now().ToNanosecond());
    c->set_seq(4);
    c->set_content("ChatterMsg");
    r.Enqueue(c);
  }
  {
    auto c = std::make_shared<Chatter>();
    c->set_timestamp(Time::Now().ToNanosecond());
    c->set_lidar_timestamp(Time::Now().ToNanosecond());
    c->set_seq(5);
    c->set_content("ChatterMsg");
    r.Enqueue(c);
  }

  EXPECT_EQ(r.GetHistoryDepth(), 3);

  auto latestMsg = r.GetLatestObserved();
  auto oldestMsg = r.GetOldestObserved();

  EXPECT_EQ(nullptr, latestMsg);
  EXPECT_EQ(nullptr, oldestMsg);
  EXPECT_EQ(r.Begin(), r.End());

  r.Observe();

  latestMsg = r.GetLatestObserved();
  oldestMsg = r.GetOldestObserved();

  EXPECT_EQ(latestMsg->seq(), 5);
  EXPECT_EQ(oldestMsg->seq(), 3);
}

}  // namespace reader
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  return RUN_ALL_TESTS();
}
