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

#include "cyber/component/component.h"

#include <memory>
#include "gtest/gtest.h"

#include "cyber/init.h"
#include "cyber/message/raw_message.h"

namespace apollo {
namespace cyber {

using apollo::cyber::Component;
using apollo::cyber::message::RawMessage;
using apollo::cyber::proto::ComponentConfig;
using apollo::cyber::proto::TimerComponentConfig;
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

TEST(CommonComponent, init) {
  ret_proc = true;
  ret_init = true;
  apollo::cyber::proto::ComponentConfig compcfg;
  auto msg_str1 = std::make_shared<message::RawMessage>();
  auto msg_str2 = std::make_shared<message::RawMessage>();
  auto msg_str3 = std::make_shared<message::RawMessage>();
  auto msg_str4 = std::make_shared<message::RawMessage>();

  compcfg.set_name("perception");
  apollo::cyber::proto::ReaderOption *read_opt = compcfg.add_readers();
  read_opt->set_channel("/perception/channel");
  auto comC = std::make_shared<Component_C<RawMessage>>();
  EXPECT_TRUE(comC->Initialize(compcfg));
  EXPECT_TRUE(comC->Process(msg_str1));

  compcfg.set_name("perception2");
  apollo::cyber::proto::ReaderOption *read_opt2 = compcfg.add_readers();
  read_opt2->set_channel("/driver/channel1");
  auto comB = std::make_shared<Component_B<RawMessage, RawMessage>>();
  EXPECT_TRUE(comB->Initialize(compcfg));
  EXPECT_TRUE(comB->Process(msg_str1, msg_str2));

  compcfg.set_name("perception3");
  apollo::cyber::proto::ReaderOption *read_opt3 = compcfg.add_readers();
  read_opt3->set_channel("/driver/channel2");
  compcfg.set_name("perception4");
  apollo::cyber::proto::ReaderOption *read_opt4 = compcfg.add_readers();
  read_opt4->set_channel("/driver/channel3");
  auto comA = std::make_shared<
      Component_A<RawMessage, RawMessage, RawMessage, RawMessage>>();
  EXPECT_TRUE(comA->Initialize(compcfg));
  EXPECT_TRUE(comA->Process(msg_str1, msg_str2, msg_str3, msg_str4));
}

TEST(CommonComponentFail, init) {
  ret_proc = false;
  ret_init = false;
  apollo::cyber::proto::ComponentConfig compcfg;

  auto msg_str1 = std::make_shared<message::RawMessage>();
  auto msg_str2 = std::make_shared<message::RawMessage>();
  auto msg_str3 = std::make_shared<message::RawMessage>();
  auto msg_str4 = std::make_shared<message::RawMessage>();

  compcfg.set_name("perception_f");
  apollo::cyber::proto::ReaderOption *read_opt = compcfg.add_readers();
  read_opt->set_channel("/perception/channel");
  auto comC = std::make_shared<Component_C<RawMessage>>();
  EXPECT_FALSE(comC->Initialize(compcfg));
  EXPECT_FALSE(comC->Process(msg_str1));

  compcfg.set_name("perception2_f");
  apollo::cyber::proto::ReaderOption *read_opt2 = compcfg.add_readers();
  read_opt2->set_channel("/driver/channel1");
  auto comB = std::make_shared<Component_B<RawMessage, RawMessage>>();
  EXPECT_FALSE(comB->Initialize(compcfg));
  EXPECT_FALSE(comB->Process(msg_str1, msg_str2));

  compcfg.set_name("perception3_f");
  apollo::cyber::proto::ReaderOption *read_opt3 = compcfg.add_readers();
  read_opt3->set_channel("/driver/channel2");
  apollo::cyber::proto::ReaderOption *read_opt4 = compcfg.add_readers();
  read_opt4->set_channel("/driver/channel3");
  auto comA = std::make_shared<
      Component_A<RawMessage, RawMessage, RawMessage, RawMessage>>();
  EXPECT_FALSE(comA->Initialize(compcfg));
  EXPECT_FALSE(comA->Process(msg_str1, msg_str2, msg_str3, msg_str4));
}

}  // namespace cyber
}  // namespace apollo

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  return RUN_ALL_TESTS();
}
