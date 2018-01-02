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
 * @brief test the Dropbox class
 */

#include <gtest/gtest.h>

#include "modules/common/util/dropbox.h"

namespace apollo {
namespace common {
namespace util {

TEST(Dropbox, case_int) {
  EXPECT_EQ(nullptr, Dropbox<int>::Open()->Get("a"));
  Dropbox<int>::Open()->Set("a", 1);
  const auto* result = Dropbox<int>::Open()->Get("a");
  EXPECT_EQ(1, *result);
  Dropbox<int>::Open()->Set("a", 2);
  result = Dropbox<int>::Open()->Get("a");
  EXPECT_EQ(2, *result);
  Dropbox<int>::Open()->Remove("a");
  EXPECT_EQ(nullptr, Dropbox<float>::Open()->Get("a"));
  EXPECT_EQ(nullptr, Dropbox<int>::Open()->Get("b"));
}

TEST(Dropbox, case_remove) {
  EXPECT_EQ(nullptr, Dropbox<int>::Open()->Get("a"));
  Dropbox<int>::Open()->Set("a", 1);
  const auto* result = Dropbox<int>::Open()->Get("a");
  EXPECT_EQ(1, *result);
  Dropbox<int>::Open()->Remove("a");
  EXPECT_EQ(nullptr, Dropbox<int>::Open()->Get("a"));
}

TEST(Dropbox, case_float) {
  EXPECT_EQ(nullptr, Dropbox<float>::Open()->Get("a"));
  Dropbox<float>::Open()->Set("a", 1.0);
  float* result = Dropbox<float>::Open()->Get("a");
  EXPECT_FLOAT_EQ(1.0, *result);
  Dropbox<float>::Open()->Set("a", 2.0);
  result = Dropbox<float>::Open()->Get("a");
  EXPECT_FLOAT_EQ(2.0, *result);
  result = Dropbox<float>::Open()->Get("b");
  EXPECT_EQ(nullptr, result);
  Dropbox<float>::Open()->Remove("a");
}

TEST(Dropbox, case_vector_int) {
  EXPECT_EQ(nullptr, Dropbox<std::vector<int>>::Open()->Get("a"));
  std::vector<int> v{1, 2, 3};
  Dropbox<decltype(v)>::Open()->Set("a", v);
  auto* result = Dropbox<std::vector<int>>::Open()->Get("a");
  EXPECT_EQ(v, *result);
  result->push_back(4);
  result = Dropbox<std::vector<int>>::Open()->Get("a");
  EXPECT_EQ(4, result->size());
  Dropbox<std::vector<int>>::Open()->Set("a", v);
  result = Dropbox<std::vector<int>>::Open()->Get("a");
  EXPECT_EQ(3, result->size());
  Dropbox<float>::Open()->Set("a", 2.0);
  const auto* float_result = Dropbox<float>::Open()->Get("a");
  EXPECT_FLOAT_EQ(2.0, *float_result);
  EXPECT_EQ(nullptr, Dropbox<float>::Open()->Get("b"));
  Dropbox<std::vector<int>>::Open()->Remove("a");
}

}  // namespace util
}  // namespace common
}  // namespace apollo
