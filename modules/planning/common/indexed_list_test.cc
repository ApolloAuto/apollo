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

#include "modules/planning/common/indexed_list.h"

#include "gtest/gtest.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using StringIndexedList = IndexedList<int, std::string>;
TEST(IndexedList, Add_ConstRef) {
  StringIndexedList object;
  {
    ASSERT_NE(nullptr, object.Add(1, "one"));
    ASSERT_NE(nullptr, object.Find(1));
    ASSERT_NE(nullptr, object.Add(1, "one"));
    const auto& items = object.Items();
    ASSERT_EQ(nullptr, object.Find(2));
    ASSERT_EQ(1, items.size());
    ASSERT_EQ("one", *items[0]);
  }
  {
    ASSERT_NE(nullptr, object.Add(2, "two"));
    ASSERT_NE(nullptr, object.Add(2, "two"));
    ASSERT_NE(nullptr, object.Find(1));
    ASSERT_NE(nullptr, object.Find(2));
    const auto& items = object.Items();
    ASSERT_EQ(2, items.size());
    ASSERT_EQ("one", *items[0]);
    ASSERT_EQ("two", *items[1]);
  }
}

TEST(IndexedList, Find) {
  StringIndexedList object;
  object.Add(1, "one");
  auto* one = object.Find(1);
  ASSERT_EQ(*one, "one");
  ASSERT_NE(nullptr, one);
  *one = "one_again";
  const auto* one_again = object.Find(1);
  ASSERT_NE(nullptr, one_again);
  ASSERT_EQ("one_again", *one_again);
  ASSERT_EQ(nullptr, object.Find(2));
}

TEST(IndexedList, Copy) {
  StringIndexedList b_object;
  b_object.Add(1, "one");
  b_object.Add(2, "two");
  StringIndexedList a_object;
  a_object.Add(3, "three");
  a_object.Add(4, "four");
  a_object = b_object;
  ASSERT_NE(nullptr, a_object.Find(1));
  ASSERT_NE(nullptr, a_object.Find(2));
  ASSERT_EQ(nullptr, a_object.Find(3));
  ASSERT_EQ(nullptr, a_object.Find(4));
}

}  // namespace planning
}  // namespace apollo
