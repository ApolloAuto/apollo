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
#include "cyber/base/for_each.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(ForEachTest, base) {
  std::vector<int> vec;
  FOR_EACH(i, 0, 100) { vec.push_back(i); }
  EXPECT_EQ(100, vec.size());

  int index = 0;
  FOR_EACH(it, vec.begin(), vec.end()) { EXPECT_EQ(index++, *it); }

  FOR_EACH(i, 0, 'a') { EXPECT_GT('a', i); }
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
