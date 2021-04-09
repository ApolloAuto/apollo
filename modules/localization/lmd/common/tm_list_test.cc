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

#include "modules/localization/lmd/common/tm_list.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

TEST(TimeMarkedListTest, InsertAndFind) {
  TimeMarkedList<int> tm(100.0);
  for (auto i = 0; i < 200; ++i) {
    tm.Push(static_cast<double>(i), i);
  }

  EXPECT_EQ(101, tm.size());
  auto latest = tm.Latest();
  EXPECT_NE(tm.end(), latest);
  if (latest != tm.end()) {
    EXPECT_EQ(199, latest->second);
  }
  auto oldest = tm.Oldest();
  EXPECT_NE(tm.end(), oldest);
  if (oldest != tm.end()) {
    EXPECT_EQ(99, oldest->second);
  }
}

TEST(TimeMarkedListTest, Older) {
  TimeMarkedList<int> tm(100.0);
  for (auto i = 0; i < 20; ++i) {
    tm.Push(static_cast<double>(i), i);
  }

  TimeMarkedList<int> tm1(100.0);
  for (auto i = 0; i < 30; ++i) {
    tm1.Push(static_cast<double>(i), i);
  }

  TimeMarkedList<int> tm2(100.0);

  EXPECT_TRUE(tm.Older(tm1));
  EXPECT_FALSE(tm1.Older(tm));
  EXPECT_FALSE(tm.Older(tm2));
  EXPECT_FALSE(tm2.Older(tm1));
}

TEST(TimeMarkedListTest, Newer) {
  TimeMarkedList<int> tm(100.0);
  for (auto i = 0; i < 20; ++i) {
    tm.Push(static_cast<double>(i), i);
  }

  TimeMarkedList<int> tm1(100.0);
  for (auto i = 0; i < 30; ++i) {
    tm1.Push(static_cast<double>(i), i);
  }

  TimeMarkedList<int> tm2(100.0);

  EXPECT_TRUE(tm1.Newer(tm));
  EXPECT_FALSE(tm.Newer(tm1));
  EXPECT_FALSE(tm2.Newer(tm));
  EXPECT_FALSE(tm1.Newer(tm2));
}

TEST(TimeMarkedListTest, RangeOf) {
  TimeMarkedList<double> tm(100.0);

  auto p = tm.RangeOf(10.0);
  EXPECT_EQ(tm.end(), p.first);
  EXPECT_EQ(tm.end(), p.second);

  for (double i = 5.0; i <= 20.0; i += 1.0) {
    tm.Push(i, i);
  }

  p = tm.RangeOf(10.5);
  EXPECT_NE(tm.end(), p.first);
  EXPECT_NE(tm.end(), p.second);
  EXPECT_NEAR(10.0, p.first->first, 1e-10);
  EXPECT_NEAR(11.0, p.second->first, 1e-10);

  p = tm.RangeOf(3.0);
  EXPECT_EQ(tm.end(), p.first);
  EXPECT_NE(tm.end(), p.second);
  EXPECT_NEAR(5.0, p.second->first, 1e-10);

  p = tm.RangeOf(30.0);
  EXPECT_NE(tm.end(), p.first);
  EXPECT_EQ(tm.end(), p.second);
  EXPECT_NEAR(20.0, p.first->first, 1e-10);

  p = tm.RangeOf(5.0);
  EXPECT_EQ(tm.end(), p.first);
  EXPECT_NE(tm.end(), p.second);
  EXPECT_NEAR(5.0, p.second->first, 1e-10);
}

TEST(TimeMarkedListTest, Nearest) {
  TimeMarkedList<double> tm(100.0);

  auto it = tm.Nearest(10.0);
  EXPECT_EQ(tm.end(), it);

  for (auto i = 5.0; i <= 20.0; i += 1.0) {
    tm.Push(i, i);
  }

  it = tm.Nearest(3.0);
  EXPECT_NE(tm.end(), it);
  EXPECT_NEAR(5.0, it->first, 1e-10);

  it = tm.Nearest(21.0);
  EXPECT_NE(tm.end(), it);
  EXPECT_NEAR(20.0, it->first, 1e-10);

  it = tm.Nearest(15.7);
  EXPECT_NE(tm.end(), it);
  EXPECT_NEAR(16.0, it->first, 1e-10);
}

}  // namespace localization
}  // namespace apollo
