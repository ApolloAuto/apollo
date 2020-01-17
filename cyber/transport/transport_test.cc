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

#include "cyber/transport/transport.h"

#include <memory>
#include <typeinfo>
#include "gtest/gtest.h"

#include "cyber/init.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace transport {

using TransmitterPtr = std::shared_ptr<Transmitter<proto::UnitTest>>;
using ReceiverPtr = std::shared_ptr<Receiver<proto::UnitTest>>;

TEST(TransportTest, constructor) {
  auto transport_a = Transport::Instance();
  auto transport_b = Transport::Instance();
  EXPECT_EQ(transport_a->participant(), transport_b->participant());
}

TEST(TransportTest, create_transmitter) {
  QosProfileConf qos_conf;
  (void)qos_conf;

  RoleAttributes attr;
  attr.set_channel_name("create_transmitter");
  Identity id;
  attr.set_id(id.HashValue());

  TransmitterPtr intra =
      Transport::Instance()->CreateTransmitter<proto::UnitTest>(
          attr, OptionalMode::INTRA);
  EXPECT_EQ(typeid(*intra), typeid(IntraTransmitter<proto::UnitTest>));

  TransmitterPtr shm =
      Transport::Instance()->CreateTransmitter<proto::UnitTest>(
          attr, OptionalMode::SHM);
  EXPECT_EQ(typeid(*shm), typeid(ShmTransmitter<proto::UnitTest>));
}

TEST(TransportTest, create_receiver) {
  RoleAttributes attr;
  attr.set_channel_name("create_receiver");
  Identity id;
  attr.set_id(id.HashValue());

  auto listener = [](const std::shared_ptr<proto::UnitTest>&,
                     const MessageInfo&, const RoleAttributes&) {};

  ReceiverPtr intra = Transport::Instance()->CreateReceiver<proto::UnitTest>(
      attr, listener, OptionalMode::INTRA);
  EXPECT_EQ(typeid(*intra), typeid(IntraReceiver<proto::UnitTest>));

  ReceiverPtr shm = Transport::Instance()->CreateReceiver<proto::UnitTest>(
      attr, listener, OptionalMode::SHM);
  EXPECT_EQ(typeid(*shm), typeid(ShmReceiver<proto::UnitTest>));
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
