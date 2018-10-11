/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/common/kv_db/kv_db.h"

#include <thread>

#include "gtest/gtest.h"

namespace apollo {
namespace common {

TEST(KVDBTest, CRUD) {
  EXPECT_TRUE(KVDB::Delete("test_key"));
  EXPECT_FALSE(KVDB::Has("test_key"));

  // Put
  EXPECT_TRUE(KVDB::Put("test_key", "val0"));
  EXPECT_TRUE(KVDB::Has("test_key"));
  EXPECT_EQ("val0", KVDB::Get("test_key"));

  // Update
  EXPECT_TRUE(KVDB::Put("test_key", "val1"));
  EXPECT_TRUE(KVDB::Has("test_key"));
  EXPECT_EQ("val1", KVDB::Get("test_key"));

  // Delete
  EXPECT_TRUE(KVDB::Delete("test_key"));
  EXPECT_FALSE(KVDB::Has("test_key"));
}

TEST(KVDBTest, GetDefault) {
  EXPECT_EQ("", KVDB::Get("test_key"));
  EXPECT_EQ("default", KVDB::Get("test_key", "default"));
}

TEST(KVDBTest, MultiThreads) {
  static const int N_THREADS = 10;

  std::vector<std::unique_ptr<std::thread>> threads(N_THREADS);
  for (auto &th : threads) {
    th.reset(new std::thread([]() {
      KVDB::Delete("test_key");
      KVDB::Has("test_key");
      KVDB::Put("test_key", "val0");
      KVDB::Get("test_key");
    }));
  }

  for (auto &th : threads) {
    th->join();
  }
}

}  // namespace common
}  // namespace apollo
