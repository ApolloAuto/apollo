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

#include "cyber/common/macros.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace common {

class ClassWithShutdown {
 public:
  void Shutdown() { set_foo(1); }

  static int foo() { return foo_; }
  static void set_foo(int val) { foo_ = val; }

 private:
  static int foo_;
  DECLARE_SINGLETON(ClassWithShutdown)
};

int ClassWithShutdown::foo_ = 0;
inline ClassWithShutdown::ClassWithShutdown() {}

class ClassWithoutShutdown {
 private:
  DECLARE_SINGLETON(ClassWithoutShutdown)
};

inline ClassWithoutShutdown::ClassWithoutShutdown() {}

TEST(MacrosTest, has_shut_down_test) {
  EXPECT_TRUE(HasShutdown<ClassWithShutdown>::value);
  EXPECT_FALSE(HasShutdown<ClassWithoutShutdown>::value);
}

TEST(MacrosTest, shut_down_test) {
  EXPECT_EQ(ClassWithShutdown::foo(), 0);
  ClassWithShutdown::CleanUp();
  EXPECT_EQ(ClassWithShutdown::foo(), 0);
  ClassWithShutdown::Instance();
  ClassWithShutdown::CleanUp();
  EXPECT_EQ(ClassWithShutdown::foo(), 1);

  ClassWithoutShutdown::CleanUp();
  ClassWithoutShutdown::Instance();
  ClassWithoutShutdown::CleanUp();
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
