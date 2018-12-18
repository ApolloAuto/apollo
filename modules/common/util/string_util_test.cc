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

TEST(StringUtilTest, Split) {
  {
    std::vector<std::string> result;
    Split("abc.def", '.', &result);
    EXPECT_EQ(result.size(), 2);
    EXPECT_EQ(result[0], "abc");
    EXPECT_EQ(result[1], "def");
  }
  {
    std::vector<std::string> result;
    Split("abc.def", 'x', &result);
    EXPECT_EQ(result.size(), 1);
    EXPECT_EQ(result[0], "abc.def");
  }
}

TEST(StringUtilTest, StartWith) {
  EXPECT_TRUE(StartWith("abc.def", ""));
  EXPECT_TRUE(StartWith("abc.def", "abc"));
  EXPECT_TRUE(StartWith("abc.def", "abc."));
  EXPECT_FALSE(StartWith("abc.def", "abcd"));
  EXPECT_FALSE(StartWith("abc.def", "bc"));
}

TEST(StringUtilTest, EndWith) {
  EXPECT_TRUE(EndWith("abc.def", ""));
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

TEST(StringUtilTest, DecodeBase64) {
  EXPECT_EQ("", DecodeBase64(""));
  EXPECT_EQ("f", DecodeBase64("Zg=="));
  EXPECT_EQ("fo", DecodeBase64("Zm8="));
  EXPECT_EQ("foo", DecodeBase64("Zm9v"));
  EXPECT_EQ("foob", DecodeBase64("Zm9vYg=="));
  EXPECT_EQ("fooba", DecodeBase64("Zm9vYmE="));
  EXPECT_EQ("foobar", DecodeBase64("Zm9vYmFy"));
}

TEST(StringUtilTest, EncodeBase64) {
  EXPECT_EQ("", EncodeBase64(""));
  EXPECT_EQ("Zg==", EncodeBase64("f"));
  EXPECT_EQ("Zm8=", EncodeBase64("fo"));
  EXPECT_EQ("Zm9v", EncodeBase64("foo"));
  EXPECT_EQ("Zm9vYg==", EncodeBase64("foob"));
  EXPECT_EQ("Zm9vYmE=", EncodeBase64("fooba"));
  EXPECT_EQ("Zm9vYmFy", EncodeBase64("foobar"));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
