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

#include "cyber/base/signal.h"

#include <memory>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(SlotTest, zero_input_param) {
  char ch = '0';
  Slot<> slot_a([&ch]() { ch = 'a'; });
  EXPECT_TRUE(slot_a.connected());

  slot_a();
  EXPECT_EQ(ch, 'a');

  slot_a.Disconnect();
  EXPECT_FALSE(slot_a.connected());

  ch = '0';
  slot_a();
  EXPECT_NE(ch, 'a');

  Slot<> slot_b([&ch]() { ch = 'b'; }, false);
  EXPECT_FALSE(slot_b.connected());

  ch = '0';
  slot_b();
  EXPECT_NE(ch, 'b');

  Slot<> slot_c(nullptr);
  EXPECT_NO_FATAL_FAILURE(slot_c());
}

TEST(SlotTest, two_input_params) {
  int sum = 0;
  Slot<int, int> slot_a([&sum](int lhs, int rhs) { sum = lhs + rhs; });
  EXPECT_TRUE(slot_a.connected());

  int lhs = 1, rhs = 2;
  slot_a(lhs, rhs);
  EXPECT_EQ(sum, lhs + rhs);

  Slot<int, int> slot_b(slot_a);
  lhs = 3;
  rhs = 4;
  slot_b(lhs, rhs);
  EXPECT_EQ(sum, lhs + rhs);

  slot_b.Disconnect();
  EXPECT_FALSE(slot_b.connected());

  sum = 0;
  lhs = 5;
  rhs = 6;
  slot_b(lhs, rhs);
  EXPECT_EQ(sum, 0);
}

TEST(ConnectionTest, null_signal) {
  Connection<> conn_a;
  EXPECT_FALSE(conn_a.IsConnected());
  EXPECT_FALSE(conn_a.Disconnect());
  EXPECT_FALSE(conn_a.HasSlot(nullptr));

  auto slot = std::make_shared<Slot<>>([]() {});
  Connection<> conn_b(slot, nullptr);
  EXPECT_TRUE(conn_b.IsConnected());
  EXPECT_FALSE(conn_b.Disconnect());
  EXPECT_TRUE(conn_b.HasSlot(slot));

  EXPECT_FALSE(conn_a.HasSlot(slot));

  conn_b = conn_b;
  conn_a = conn_b;
  EXPECT_TRUE(conn_a.IsConnected());
  EXPECT_FALSE(conn_a.Disconnect());
  EXPECT_TRUE(conn_a.HasSlot(slot));

  Signal<> sig;
  Connection<> conn_c(nullptr, &sig);
  EXPECT_FALSE(conn_c.Disconnect());
}

TEST(SignalTest, module) {
  Signal<int, int> sig;

  int sum_a = 0;
  auto conn_a = sig.Connect([&sum_a](int lhs, int rhs) { sum_a = lhs + rhs; });

  int sum_b = 0;
  auto conn_b = sig.Connect([&sum_b](int lhs, int rhs) { sum_b = lhs + rhs; });

  int lhs = 1, rhs = 2;
  sig(lhs, rhs);
  EXPECT_EQ(sum_a, lhs + rhs);
  EXPECT_EQ(sum_b, lhs + rhs);

  Connection<int, int> conn_c;
  EXPECT_FALSE(sig.Disconnect(conn_c));
  EXPECT_TRUE(sig.Disconnect(conn_b));
  sum_a = 0;
  sum_b = 0;
  lhs = 3;
  rhs = 4;
  sig(lhs, rhs);
  EXPECT_EQ(sum_a, lhs + rhs);
  EXPECT_NE(sum_b, lhs + rhs);

  sig.DisconnectAllSlots();
  sum_a = 0;
  sum_b = 0;
  lhs = 5;
  rhs = 6;
  sig(lhs, rhs);
  EXPECT_NE(sum_a, lhs + rhs);
  EXPECT_NE(sum_b, lhs + rhs);
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
