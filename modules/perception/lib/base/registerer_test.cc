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

#include "modules/perception/lib/base/registerer.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {

class BaseClass {
 public:
  BaseClass() = default;
  ~BaseClass() = default;
  virtual std::string name() const = 0;
};
REGISTER_REGISTERER(BaseClass);
#define REGISTER_TEST(name) REGISTER_CLASS(BaseClass, name)

class DerivedClass1 : BaseClass {
 public:
  DerivedClass1() = default;
  ~DerivedClass1() = default;
  virtual std::string name() const {
    return "DerivedClass1";
  }
};
REGISTER_TEST(DerivedClass1);

class DerivedClass2 : BaseClass {
 public:
  DerivedClass2() = default;
  ~DerivedClass2() = default;
  virtual std::string name() const {
    return "DerivedClass2";
  }
};
REGISTER_TEST(DerivedClass2);

TEST(RegistererTest, test) {
  RegisterFactoryDerivedClass1();
  RegisterFactoryDerivedClass2();
  BaseClass* ptr = nullptr;
  ptr = BaseClassRegisterer::GetInstanceByName("DerivedClass1");
  ASSERT_TRUE(ptr != nullptr);
  EXPECT_EQ(ptr->name(), "DerivedClass1");
  ptr = BaseClassRegisterer::GetInstanceByName("DerivedClass2");
  ASSERT_TRUE(ptr != nullptr);
  EXPECT_EQ(ptr->name(), "DerivedClass2");

  ptr = BaseClassRegisterer::GetInstanceByName("NotExists");
  ASSERT_TRUE(ptr == nullptr);

  std::vector<std::string> derived_classes;
  EXPECT_TRUE(GetRegisteredClasses("BaseClass", &derived_classes));
  EXPECT_FALSE(GetRegisteredClasses("NotExitstClass", &derived_classes));
  EXPECT_EQ(derived_classes.size(), 2u);
  EXPECT_TRUE(derived_classes[0] == "DerivedClass1" ||
              derived_classes[0] == "DerivedClass2");
  EXPECT_TRUE(derived_classes[1] == "DerivedClass1" ||
              derived_classes[1] == "DerivedClass2");
}

TEST(ObjectFactoryTest, test_ObjectFactory) {
  ObjectFactory obj_fac;
  Any any = obj_fac.NewInstance();
  EXPECT_TRUE(any.content_ == NULL);
  int value = 100;
  Any any2(value);
  EXPECT_FALSE(any2.content_ == NULL);
  EXPECT_EQ(*any2.AnyCast<int>(), value);
  Any any3(any2);
  EXPECT_FALSE(any3.content_ == NULL);
  EXPECT_EQ(*any3.AnyCast<int>(), value);
}

}  // namespace perception
}  // namespace apollo
