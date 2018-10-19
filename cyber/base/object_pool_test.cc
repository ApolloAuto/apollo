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

namespace apollo {
namespace cyber {
namespace base {

class TestNode {
 public:
  TestNode() {}
  explicit TestNode(int data) : value(data) {}
  int value = 0;
};

TEST(ObjectPoolTest, get_object) {
  auto pool = new ObjectPool<TestNode>(10, 100);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(100, pool->GetObject()->value);
  }
  EXPECT_NE(nullptr, pool->GetObject());
  delete pool;

  auto pool2 = new ObjectPool<TestNode>(10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(0, pool2->GetObject()->value);
  }
  EXPECT_NE(nullptr, pool2->GetObject());
  delete pool2;
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
