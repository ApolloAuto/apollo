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

#include <typeinfo>

#include "cybertron/proto/unit_test.pb.h"
#include "cybertron/transport/common/identity.h"
#include "cybertron/transport/transport.h"

namespace apollo {
namespace cybertron {
namespace transport {

using UpperReachPtr = std::shared_ptr<UpperReach<proto::UnitTest>>;
using LowerReachPtr = std::shared_ptr<LowerReach<proto::UnitTest>>;

TEST(TransportTest, constructor) {
  Transport transport_a;
  Transport transport_b;
  EXPECT_EQ(transport_a.participant(), transport_b.participant());
}

TEST(TransportTest, create_upper_reach) {
  QosProfileConf qos_conf;
  (void)qos_conf;

  RoleAttributes attr;
  attr.set_channel_name("create_upper_reach");
  Identity id;
  attr.set_id(id.HashValue());

  UpperReachPtr intra =
      Transport::CreateUpperReach<proto::UnitTest>(attr, OptionalMode::INTRA);
  EXPECT_EQ(typeid(*intra), typeid(IntraUpperReach<proto::UnitTest>));

  UpperReachPtr shm =
      Transport::CreateUpperReach<proto::UnitTest>(attr, OptionalMode::SHM);
  EXPECT_EQ(typeid(*shm), typeid(ShmUpperReach<proto::UnitTest>));

  UpperReachPtr rtps =
      Transport::CreateUpperReach<proto::UnitTest>(attr, OptionalMode::RTPS);
  EXPECT_EQ(typeid(*rtps), typeid(RtpsUpperReach<proto::UnitTest>));

  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  UpperReachPtr hybrid = Transport::CreateUpperReach<proto::UnitTest>(attr);
  EXPECT_EQ(typeid(*hybrid), typeid(HybridUpperReach<proto::UnitTest>));
}

TEST(TransportTest, create_lower_reach) {
  RoleAttributes attr;
  attr.set_channel_name("create_lower_reach");
  Identity id;
  attr.set_id(id.HashValue());

  auto listener = [](const std::shared_ptr<proto::UnitTest>&,
                     const MessageInfo&, const RoleAttributes&) {};

  LowerReachPtr intra = Transport::CreateLowerReach<proto::UnitTest>(
      attr, listener, OptionalMode::INTRA);
  EXPECT_EQ(typeid(*intra), typeid(IntraLowerReach<proto::UnitTest>));

  LowerReachPtr shm = Transport::CreateLowerReach<proto::UnitTest>(
      attr, listener, OptionalMode::SHM);
  EXPECT_EQ(typeid(*shm), typeid(ShmLowerReach<proto::UnitTest>));

  LowerReachPtr rtps = Transport::CreateLowerReach<proto::UnitTest>(
      attr, listener, OptionalMode::RTPS);
  EXPECT_EQ(typeid(*rtps), typeid(RtpsLowerReach<proto::UnitTest>));

  attr.mutable_qos_profile()->CopyFrom(QosProfileConf::QOS_PROFILE_DEFAULT);
  LowerReachPtr hybrid =
      Transport::CreateLowerReach<proto::UnitTest>(attr, listener);
  EXPECT_EQ(typeid(*hybrid), typeid(HybridLowerReach<proto::UnitTest>));
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
