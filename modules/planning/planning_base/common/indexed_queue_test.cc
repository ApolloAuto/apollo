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

/**
 * @file
 **/

#include "modules/planning/planning_base/common/indexed_queue.h"

#include "gtest/gtest.h"

#include "modules/common/util/future.h"

namespace apollo {
namespace planning {

using StringIndexedQueue = IndexedQueue<int, std::string>;
TEST(IndexedQueue, QueueSize1) {
  StringIndexedQueue object(1);
  ASSERT_TRUE(object.Add(1, std::make_unique<std::string>("one")));
  ASSERT_TRUE(object.Find(1) != nullptr);
  ASSERT_TRUE(object.Find(2) == nullptr);
  ASSERT_FALSE(object.Add(1, std::make_unique<std::string>("one")));
  ASSERT_EQ("one", *object.Latest());
  ASSERT_TRUE(object.Add(2, std::make_unique<std::string>("two")));
  ASSERT_TRUE(object.Find(1) == nullptr);
  ASSERT_TRUE(object.Find(2) != nullptr);
  ASSERT_EQ("two", *object.Latest());
}

TEST(IndexedQueue, QueueSize2) {
  StringIndexedQueue object(2);
  ASSERT_TRUE(object.Add(1, std::make_unique<std::string>("one")));
  ASSERT_TRUE(object.Find(1) != nullptr);
  ASSERT_TRUE(object.Find(2) == nullptr);
  ASSERT_FALSE(object.Add(1, std::make_unique<std::string>("one")));
  ASSERT_EQ("one", *object.Latest());
  ASSERT_TRUE(object.Add(2, std::make_unique<std::string>("two")));
  ASSERT_TRUE(object.Find(1) != nullptr);
  ASSERT_TRUE(object.Find(2) != nullptr);
  ASSERT_EQ("two", *object.Latest());
  ASSERT_TRUE(object.Add(3, std::make_unique<std::string>("three")));
  ASSERT_TRUE(object.Find(1) == nullptr);
  ASSERT_TRUE(object.Find(2) != nullptr);
  ASSERT_TRUE(object.Find(3) != nullptr);
  ASSERT_TRUE(object.Find(4) == nullptr);
  ASSERT_EQ("three", *object.Latest());
}

}  // namespace planning
}  // namespace apollo
