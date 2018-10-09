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

#include "cybertron/component/component.h"
#include "cybertron/init.h"
#include "cybertron/message/raw_message.h"

namespace apollo {
namespace cybertron {

using apollo::cybertron::Component;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::proto::ComponentConfig;
using apollo::cybertron::proto::TimerComponentConfig;
static bool ret_proc = true;
static bool ret_init = true;
template <typename M0, typename M1 = NullType, typename M2 = NullType,
          typename M3 = NullType>
class Component_A : public Component<M0, M1, M2, M3> {
 public:
  Component_A() {}
  bool Init() { return ret_init; }

 private:
  bool Proc(const std::shared_ptr<M0> &msg0, const std::shared_ptr<M1> &msg1,
            const std::shared_ptr<M2> &msg2, const std::shared_ptr<M3> &msg3) {
    return ret_proc;
  }
};

template <typename M0, typename M1>
class Component_B : public Component<M0, M1> {
 public:
  Component_B() {}
  bool Init() { return ret_init; }

 private:
  bool Proc(const std::shared_ptr<M0> &, const std::shared_ptr<M1> &) {
    return ret_proc;
  }
};

template <typename M0>
class Component_C : public Component<M0> {
 public:
  Component_C() {}
  bool Init() { return ret_init; }

 private:
  bool Proc(const std::shared_ptr<M0> &) { return ret_proc; }
};

TEST(TimerComponent, init) {
  ret_proc = true;
  ret_init = true;
  apollo::cybertron::proto::ComponentConfig compcfg;
  auto msg_str1 = std::make_shared<message::RawMessage>();
  auto msg_str2 = std::make_shared<message::RawMessage>();
  auto msg_str3 = std::make_shared<message::RawMessage>();
  auto msg_str4 = std::make_shared<message::RawMessage>();

  compcfg.set_name("perception");
  apollo::cybertron::proto::ReaderOption *read_opt = compcfg.add_readers();
  read_opt->set_channel("/perception/channel");
  auto comC = std::make_shared<Component_C<RawMessage>>();
  EXPECT_EQ(true, comC->Initialize(compcfg));
  EXPECT_EQ(true, comC->Process(msg_str1));

  compcfg.set_name("perception2");
  apollo::cybertron::proto::ReaderOption *read_opt2 = compcfg.add_readers();
  read_opt2->set_channel("/driver/channel1");
  auto comB = std::make_shared<Component_B<RawMessage, RawMessage>>();
  EXPECT_EQ(true, comB->Initialize(compcfg));
  EXPECT_EQ(true, comB->Process(msg_str1, msg_str2));

  compcfg.set_name("perception3");
  apollo::cybertron::proto::ReaderOption *read_opt3 = compcfg.add_readers();
  read_opt3->set_channel("/driver/channel2");
  compcfg.set_name("perception4");
  apollo::cybertron::proto::ReaderOption *read_opt4 = compcfg.add_readers();
  read_opt4->set_channel("/driver/channel3");
  auto comA = std::make_shared<
      Component_A<RawMessage, RawMessage, RawMessage, RawMessage>>();
  EXPECT_EQ(true, comA->Initialize(compcfg));
  EXPECT_EQ(true, comA->Process(msg_str1, msg_str2, msg_str3, msg_str4));
}

TEST(TimerComponentFail, init) {
  ret_proc = false;
  ret_init = false;
  apollo::cybertron::proto::ComponentConfig compcfg;

  auto msg_str1 = std::make_shared<message::RawMessage>();
  auto msg_str2 = std::make_shared<message::RawMessage>();
  auto msg_str3 = std::make_shared<message::RawMessage>();
  auto msg_str4 = std::make_shared<message::RawMessage>();

  compcfg.set_name("perception_f");
  apollo::cybertron::proto::ReaderOption *read_opt = compcfg.add_readers();
  read_opt->set_channel("/perception/channel");
  auto comC = std::make_shared<Component_C<RawMessage>>();
  EXPECT_EQ(false, comC->Initialize(compcfg));
  EXPECT_EQ(false, comC->Process(msg_str1));

  compcfg.set_name("perception2_f");
  apollo::cybertron::proto::ReaderOption *read_opt2 = compcfg.add_readers();
  read_opt2->set_channel("/driver/channel");
  auto comB = std::make_shared<Component_B<RawMessage, RawMessage>>();
  EXPECT_EQ(false, comB->Initialize(compcfg));
  EXPECT_EQ(false, comB->Process(msg_str1, msg_str2));

  compcfg.set_name("perception3_F");
  apollo::cybertron::proto::ReaderOption *read_opt3 = compcfg.add_readers();
  read_opt3->set_channel("/driver/channel");
  auto comA = std::make_shared<
      Component_A<RawMessage, RawMessage, RawMessage, RawMessage>>();
  EXPECT_EQ(false, comA->Initialize(compcfg));
  EXPECT_EQ(false, comA->Process(msg_str1, msg_str2, msg_str3, msg_str4));
}

}  // namespace cybertron
}  // namespace apollo

int main(int argc, char **argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
