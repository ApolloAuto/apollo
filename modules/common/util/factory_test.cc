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
#include <string>
#include "gtest/gtest.h"
#include "modules/common/util/factory.h"

namespace apollo {
namespace common {
namespace util {

class Base {
 public:
  virtual std::string Name() const { return "base"; }
};

class Derived : public Base {
 public:
  virtual std::string Name() const { return "derived"; }
};

TEST(FactoryTest, Register) {
  Factory<std::string, Base> factory;
  EXPECT_TRUE(factory.Register("derived_class",
                               []() -> Base* { return new Derived(); }));
  auto derived_ptr = factory.CreateObject("derived_class");
  EXPECT_TRUE(derived_ptr != nullptr);
  EXPECT_EQ("derived", derived_ptr->Name());
  auto non_exist_ptr = factory.CreateObject("non_exist_class");
  EXPECT_TRUE(non_exist_ptr == nullptr);
}

TEST(FactoryTest, Unregister) {
  Factory<std::string, Base> factory;
  EXPECT_TRUE(factory.Register("derived_class",
                               []() -> Base* { return new Derived(); }));
  EXPECT_FALSE(factory.Unregister("fake_class"));
  auto derived_ptr = factory.CreateObject("derived_class");
  EXPECT_TRUE(derived_ptr != nullptr);
  EXPECT_TRUE(factory.Unregister("derived_class"));
  auto non_exist_ptr = factory.CreateObject("derived_class");
  EXPECT_TRUE(non_exist_ptr == nullptr);
}
}  // namespace util
}  // namespace common
}  // namespace apollo
