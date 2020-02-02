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

#include "cyber/base/object_pool.h"

#include <thread>
#include "gtest/gtest.h"

#include "cyber/base/concurrent_object_pool.h"

namespace apollo {
namespace cyber {
namespace base {

struct TestNode {
  TestNode() : inited(true) {}
  ~TestNode() {
    inited = false;
    value = 1;
  }
  explicit TestNode(int data) : value(data), inited(true) {}
  int value = 0;
  bool inited = false;
};

TEST(CCObjectPoolTest, base) {
  const uint32_t capacity = 1024;
  std::vector<std::shared_ptr<TestNode>> vec;
  vec.reserve(capacity);
  auto pool = std::make_shared<CCObjectPool<TestNode>>(capacity);

  FOR_EACH(i, 0, capacity) {
    auto obj = pool->ConstructObject(i);
    vec.push_back(obj);
    EXPECT_EQ(i, obj->value);
  }

  FOR_EACH(i, 0, 10) { EXPECT_EQ(nullptr, pool->ConstructObject(10)); }

  vec.clear();
  EXPECT_EQ(10, pool->ConstructObject(10)->value);

  pool.reset();
}

TEST(CCObjectPoolTest, multi_thread) {
  const uint32_t capacity = 1024;
  std::vector<std::shared_ptr<TestNode>> vec;
  std::vector<std::thread> thread_pool;
  vec.reserve(capacity);

  auto pool = std::make_shared<CCObjectPool<TestNode>>(capacity);
  FOR_EACH(i, 0, 16) {
    thread_pool.emplace_back([pool]() {
      FOR_EACH(i, 0, 100000) { pool->ConstructObject(10); }
    });
  }
  for (auto& thread : thread_pool) {
    thread.join();
  }

  FOR_EACH(i, 0, capacity) {
    auto obj = pool->ConstructObject(i);
    vec.push_back(obj);
    EXPECT_EQ(i, obj->value);
  }

  FOR_EACH(i, 0, 10) { EXPECT_EQ(nullptr, pool->ConstructObject(10)); }
  vec.clear();
}

TEST(CCObjectPoolTest, construct_object) {
  const uint32_t capacity = 1024;
  auto pool = std::make_shared<CCObjectPool<TestNode>>(capacity);
  std::vector<std::shared_ptr<TestNode>> vec;

  FOR_EACH(i, 0, capacity) {
    auto obj = pool->ConstructObject(i);
    vec.push_back(obj);
    EXPECT_TRUE(obj->inited);
    EXPECT_EQ(i, obj->value);
  }
  vec.clear();
}

TEST(CCObjectPoolTest, construct_all) {
  const uint32_t capacity = 1024;
  std::vector<std::shared_ptr<TestNode>> vec;
  auto pool = std::make_shared<CCObjectPool<TestNode>>(capacity);
  pool->ConstructAll();

  FOR_EACH(i, 0, capacity) {
    auto obj = pool->GetObject();
    vec.push_back(obj);
    EXPECT_TRUE(obj->inited);
  }
  vec.clear();
}

TEST(ObjectPoolTest, get_object) {
  auto pool = std::make_shared<ObjectPool<TestNode>>(100, 10);
  FOR_EACH(i, 0, 10) { EXPECT_EQ(10, pool->GetObject()->value); }
  EXPECT_NE(nullptr, pool->GetObject());
  pool.reset();

  auto pool2 = std::make_shared<ObjectPool<TestNode>>(10);
  FOR_EACH(i, 0, 10) { EXPECT_EQ(0, pool2->GetObject()->value); }
  EXPECT_NE(nullptr, pool2->GetObject());
  pool2.reset();
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
