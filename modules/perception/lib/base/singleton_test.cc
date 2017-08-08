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

#include "modules/perception/lib/base/singleton.h"
#include "gtest/gtest.h"

namespace apollo {
namespace perception {

class SingletonClass {
 public:
  int GetValue() const {
    return value_;
  }

  void SetValue(int value) {
    value_ = value;
  }

 private:
  SingletonClass() : value_(0) {}
  ~SingletonClass() {}
  friend class Singleton<SingletonClass>;

  int value_;
};

TEST(SingletonTest, test) {
  SingletonClass* clas1 = Singleton<SingletonClass>::Get();
  SingletonClass* clas2 = Singleton<SingletonClass>::Get();
  EXPECT_EQ(clas1, clas2);

  clas1->SetValue(10);
  EXPECT_EQ(clas1->GetValue(), clas2->GetValue());
}

}  // namespace perception
}  // namespace apollo
