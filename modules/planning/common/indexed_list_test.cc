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

#include <memory>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/util/util.h"
#include "modules/planning/common/indexed_list.h"

namespace apollo {
namespace planning {

using StringIndexedList = IndexedList<int, std::string>;
TEST(IndexedList, Add_UniquePtr) {
  StringIndexedList object;
  {
    ASSERT_TRUE(object.Add(1, common::util::make_unique<std::string>("one")));
    ASSERT_TRUE(object.Find(1) != nullptr);
    const auto& items = object.Items();
    ASSERT_TRUE(object.Find(2) == nullptr);
    ASSERT_FALSE(object.Add(1, common::util::make_unique<std::string>("one")));
    ASSERT_EQ(1, items.size());
    ASSERT_EQ("one", *items[0]);
  }
  {
    ASSERT_TRUE(object.Add(2, common::util::make_unique<std::string>("two")));
    ASSERT_FALSE(object.Add(2, common::util::make_unique<std::string>("two")));
    ASSERT_TRUE(object.Find(1) != nullptr);
    ASSERT_TRUE(object.Find(2) != nullptr);
    const auto& items = object.Items();
    ASSERT_EQ(2, items.size());
    ASSERT_EQ("one", *items[0]);
    ASSERT_EQ("two", *items[1]);
  }
}

TEST(IndexedList, Add_ConstRef) {
  StringIndexedList object;
  {
    ASSERT_TRUE(object.Add(1, "one"));
    ASSERT_TRUE(object.Find(1) != nullptr);
    const auto& items = object.Items();
    ASSERT_TRUE(object.Find(2) == nullptr);
    ASSERT_FALSE(object.Add(1, "one"));
    ASSERT_EQ(1, items.size());
    ASSERT_EQ("one", *items[0]);
  }
  {
    ASSERT_TRUE(object.Add(2, "two"));
    ASSERT_FALSE(object.Add(2, "two"));
    ASSERT_TRUE(object.Find(1) != nullptr);
    ASSERT_TRUE(object.Find(2) != nullptr);
    const auto& items = object.Items();
    ASSERT_EQ(2, items.size());
    ASSERT_EQ("one", *items[0]);
    ASSERT_EQ("two", *items[1]);
  }
}

TEST(IndexedList, Find) {
  StringIndexedList object;
  ASSERT_TRUE(object.Add(1, common::util::make_unique<std::string>("one")));
  auto* one = object.Find(1);
  ASSERT_EQ(*one, "one");
  ASSERT_TRUE(one != nullptr);
  *one = "one_again";
  const auto* one_again = object.Find(1);
  ASSERT_TRUE(one_again != nullptr);
  ASSERT_EQ("one_again", *one_again);
  ASSERT_FALSE(object.Find(2));
}

}  // namespace planning
}  // namespace apollo
