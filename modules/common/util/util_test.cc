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

#include "modules/common/util/util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

TEST(Util, MaxElement) {
  EXPECT_EQ(3, MaxElement(std::vector<int>{1, 2, 3}));
  EXPECT_FLOAT_EQ(3.3, MaxElement(std::vector<float>{1.1, 2.2, 3.3}));
}

TEST(Util, MinElement) {
  EXPECT_EQ(1, MinElement(std::vector<int>{1, 2, 3}));
  EXPECT_FLOAT_EQ(1.1, MinElement(std::vector<float>{1.1, 2.2, 3.3}));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
