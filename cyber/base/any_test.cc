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

#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "cyber/base/any.h"
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace data {
class Foo {
 public:
  int a;
};

class Bar {
 public:
  int a;
};

class Container {
 public:
  Foo foo;
  Bar bar;
};

TEST(AnyTest, construct_test) {
  Foo foo;
  foo.a = 5;
  Any any(foo);
  EXPECT_EQ(AnyCast<Foo>(any).a, 5);

  Any any_1(any);
  EXPECT_EQ(AnyCast<Foo>(any_1).a, 5);

  Any any_2(std::move(any));
  EXPECT_EQ(AnyCast<Foo>(any_2).a, 5);

  Any any_3;
  any_3 = any_2;
  EXPECT_EQ(AnyCast<Foo>(any_3).a, 5);
}

TEST(AnyTest, empty_test) {
  Any any;
  EXPECT_EQ(any.Empty(), true);

  Any any_2(std::string("content"));
  any = any_2;
  EXPECT_EQ(any.Empty(), false);
  any_2 = any_2;
  EXPECT_EQ("content", *any_2.GetValue<std::string>());

  Any any_1 = std::move(any);
  EXPECT_EQ(any_1.Empty(), false);
  EXPECT_EQ(any.Empty(), true);

  Any any_3 = std::move(any);
  EXPECT_TRUE(any_3.Empty());
}

TEST(AnyTest, get_value_test) {
  const Any empty_any_1;
  EXPECT_EQ(nullptr, empty_any_1.GetValue<int>());
  const Any any_1(5);
  EXPECT_EQ(nullptr, any_1.GetValue<std::string>());
  EXPECT_EQ(5, *any_1.GetValue<int>());

  Any empty_any_2;
  EXPECT_EQ(nullptr, empty_any_2.GetValue<int>());
  Any any_2(5);
  EXPECT_EQ(nullptr, any_2.GetValue<std::string>());
  EXPECT_EQ(5, *any_2.GetValue<int>());
}

TEST(AnyTest, cast_test) {
  Foo foo;
  foo.a = 5;

  const Any* const_null_any = nullptr;
  EXPECT_EQ(nullptr, AnyCast<int>(const_null_any));
  const Any const_int_any(5);
  EXPECT_EQ(5, *AnyCast<int>(&const_int_any));

  Any* null_any = nullptr;
  EXPECT_EQ(nullptr, AnyCast<int>(null_any));
  Any int_any(5);
  EXPECT_EQ(5, *AnyCast<int>(&int_any));

  Any empty_any;
  EXPECT_THROW(AnyCast<int>(empty_any), BadAnyCast);

  Any any(foo);
  EXPECT_EQ(AnyCast<Foo>(any).a, 5);
  EXPECT_THROW(AnyCast<int>(any), BadAnyCast);

  Any* any_ptr = &any;
  EXPECT_EQ(AnyCast<Foo>(any_ptr)->a, 5);

  const Any* any_const_ptr = &any;
  EXPECT_EQ(AnyCast<Foo>(any_const_ptr)->a, 5);
  auto ptr = AnyCast<double>(any_const_ptr);
  EXPECT_EQ(ptr, nullptr);

  EXPECT_EQ(5, AnyCast<Foo>(std::move(any)).a);
  EXPECT_THROW(AnyCast<int>(std::move(empty_any)), BadAnyCast);
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo
