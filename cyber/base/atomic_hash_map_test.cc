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

#include "cyber/base/atomic_hash_map.h"
#include <string>
#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(AtomicHashMapTest, int_int) {
  AtomicHashMap<int, int> map;
  int value = 0;
  for (int i = 0; i < 1000; i++) {
    map.Set(i, i);
    EXPECT_TRUE(map.Has(i));
    EXPECT_TRUE(map.Get(i, &value));
    EXPECT_EQ(i, value);
    EXPECT_TRUE(map.Remove(i));
    EXPECT_FALSE(map.Has(i));
    EXPECT_FALSE(map.Get(i, &value));
    EXPECT_FALSE(map.Remove(i));
  }

  for (int i = 0; i < 1000; i++) {
    map.Set(1000 - i, i);
    EXPECT_TRUE(map.Has(1000 - i));
    EXPECT_TRUE(map.Get(1000 - i, &value));
    EXPECT_EQ(i, value);
    EXPECT_TRUE(map.Remove(1000 - i));
    EXPECT_FALSE(map.Has(1000 - i));
    EXPECT_FALSE(map.Get(1000 - i, &value));
    EXPECT_FALSE(map.Remove(1000 - i));
  }
}

TEST(AtomicHashMapTest, int_str) {
  AtomicHashMap<int, std::string> map;
  std::string value("");
  for (int i = 0; i < 1000; i++) {
    map.Set(i, std::to_string(i));
    EXPECT_TRUE(map.Has(i));
    EXPECT_TRUE(map.Get(i, &value));
    EXPECT_EQ(std::to_string(i), value);
    EXPECT_TRUE(map.Remove(i));
    EXPECT_FALSE(map.Has(i));
    EXPECT_FALSE(map.Get(i, &value));
    EXPECT_FALSE(map.Remove(i));
  }
}

TEST(AtomicHashMapTest, concurrency) {}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
