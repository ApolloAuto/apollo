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
  }

  for (int i = 0; i < 1000; i++) {
    map.Set(1000 - i, i);
    EXPECT_TRUE(map.Has(1000 - i));
    EXPECT_TRUE(map.Get(1000 - i, &value));
    EXPECT_EQ(i, value);
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
  }
  map.Set(100);
  EXPECT_TRUE(map.Get(100, &value));
  EXPECT_TRUE(value.empty());
  map.Set(100, std::move(std::string("test")));
  EXPECT_TRUE(map.Get(100, &value));
  EXPECT_EQ("test", value);
}

TEST(AtomicHashMapTest, concurrency) {
  AtomicHashMap<int, std::string, 128> map;
  int thread_num = 32;
  std::thread t[32];
  volatile bool ready = false;

  for (int i = 0; i < thread_num; i++) {
    t[i] = std::thread([&, i]() {
      while (!ready) {
#if defined(__aarch64__)
        asm volatile("yield" ::: "memory");
#else
        asm volatile("rep; nop" ::: "memory");
#endif
      }
      for (int j = 0; j < thread_num * 1024; j++) {
        auto j_str = std::to_string(j);
        map.Set(j);
        map.Set(j, j_str);
        map.Set(j, std::move(std::to_string(j)));
      }
    });
  }
  ready = true;
  for (int i = 0; i < thread_num; i++) {
    t[i].join();
  }

  std::string value("");
  for (int i = 1; i < thread_num * 1000; i++) {
    EXPECT_TRUE(map.Get(i, &value));
    EXPECT_EQ(std::to_string(i), value);
  }
  std::string* str = nullptr;
  EXPECT_TRUE(map.Get(0, &str));
  EXPECT_EQ("0", *str);
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
