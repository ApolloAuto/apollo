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

#include "modules/common/util/string_util.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

TEST(StringUtilTest, EndWith) {
  EXPECT_TRUE(EndWith("abc.def", "def"));
  EXPECT_TRUE(EndWith("abc.def", ".def"));
  EXPECT_FALSE(EndWith("abc.def", "abc"));
  EXPECT_FALSE(EndWith("abc.def", "de"));
}

TEST(StringUtilTest, IterPrinter) {
  // Container.
  std::vector<std::string> vec;
  EXPECT_EQ("", PrintIter(vec));  // Empty string
  vec.assign({"0", "1", "2"});
  EXPECT_EQ("0 1 2", PrintIter(vec));
  EXPECT_EQ("0|1|2", PrintIter(vec, "|"));
  EXPECT_EQ("0, 1, 2", PrintIter(vec.begin(), vec.end(), ", "));
  EXPECT_EQ("1", PrintIter(vec.begin() + 1, vec.end() - 1, " "));

  // Array.
  int data[] = {0, 1, 2};
  EXPECT_EQ("0 1 2", PrintIter(data));
  EXPECT_EQ("0, 1", PrintIter(data, data + 2, ", "));
  EXPECT_EQ("1", PrintIter(data + 1, data + 2, ", "));
}

TEST(StringUtilTest, Base64Decode) {
  EXPECT_EQ("", Base64Decode(""));
  EXPECT_EQ("f", Base64Decode("Zg=="));
  EXPECT_EQ("fo", Base64Decode("Zm8="));
  EXPECT_EQ("foo", Base64Decode("Zm9v"));
  EXPECT_EQ("foob", Base64Decode("Zm9vYg=="));
  EXPECT_EQ("fooba", Base64Decode("Zm9vYmE="));
  EXPECT_EQ("foobar", Base64Decode("Zm9vYmFy"));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
