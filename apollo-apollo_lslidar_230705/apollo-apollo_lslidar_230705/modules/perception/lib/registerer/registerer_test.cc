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

#include "gtest/gtest.h"

#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace lib {

class BaseClass {
 public:
  BaseClass() = default;
  ~BaseClass() = default;
  virtual std::string Name() const { return "BaseClass1"; }
};
PERCEPTION_REGISTER_REGISTERER(BaseClass);
#define PERCEPTION_REGISTER_TEST(name) \
  PERCEPTION_REGISTER_CLASS(BaseClass, name)

class DerivedClass1 : BaseClass {
 public:
  DerivedClass1() = default;
  ~DerivedClass1() = default;
  virtual std::string Name() const { return "DerivedClass1"; }
};
PERCEPTION_REGISTER_TEST(DerivedClass1);

TEST(RegistererTest, Test) {
  BaseClass* ptr = nullptr;
  ptr = BaseClassRegisterer::GetInstanceByName("DerivedClass1");
  ASSERT_TRUE(ptr != nullptr);
  EXPECT_EQ(ptr->Name(), "DerivedClass1");
  ptr = BaseClassRegisterer::GetInstanceByName("NotExists");
  ASSERT_TRUE(ptr == nullptr);
  EXPECT_TRUE(BaseClassRegisterer::IsValid("DerivedClass1"));
  EXPECT_FALSE(BaseClassRegisterer::IsValid("NotExists"));
  EXPECT_EQ(BaseClassRegisterer::GetUniqInstanceName(), "DerivedClass1");
  BaseClass* ptr1 = BaseClassRegisterer::GetUniqInstance();
  EXPECT_NE(ptr1, nullptr);
  std::vector<std::string> derived_classes;
  EXPECT_TRUE(GetRegisteredClasses("BaseClass", &derived_classes));
  EXPECT_FALSE(GetRegisteredClasses("BaseClass2", &derived_classes));
  EXPECT_EQ(derived_classes.size(), 1u);
  EXPECT_EQ(derived_classes[0], "DerivedClass1");
  ObjectFactoryDerivedClass1 obj_factory_drived1;
  obj_factory_drived1.NewInstance();
  Any any;
  // TODO(all) enable this check
  // EXPECT_EQ(any.content_, nullptr);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
