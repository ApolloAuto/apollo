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
#include <gtest/gtest.h>

#include "modules/perception/lib/utils/string_util.h"

namespace apollo {
namespace perception {
namespace lib {

using std::string;
using std::vector;

TEST(StringUtilTest, TestExplode) {
  vector<string> terms_vec;
  StringUtil::Explode("a,b,c,", ',', &terms_vec);
  ASSERT_EQ(3u, terms_vec.size());
  EXPECT_EQ("a", terms_vec[0]);
  EXPECT_EQ("b", terms_vec[1]);
  EXPECT_EQ("c", terms_vec[2]);

  // reset
  terms_vec.clear();

  StringUtil::Explode("$abc$edf$ghi#.pos", '$', &terms_vec);
  ASSERT_EQ(3u, terms_vec.size());
  EXPECT_EQ("abc", terms_vec[0]);
  EXPECT_EQ("edf", terms_vec[1]);
  EXPECT_EQ("ghi#.pos", terms_vec[2]);

  // reset
  terms_vec.clear();

  StringUtil::Explode("/dir/123456#abc", '$', &terms_vec);
  ASSERT_EQ(1u, terms_vec.size());
  EXPECT_EQ("/dir/123456#abc", terms_vec[0]);
}

TEST(StringUtilTest, TestTrim1) {
  string string01 = "   abc edf ";
  StringUtil::Trim(TRIM_LEFT, &string01);
  EXPECT_EQ("abc edf ", string01);

  string string02 = "  abc edf g  ";
  StringUtil::Trim(TRIM_RIGHT, &string02);
  EXPECT_EQ("  abc edf g", string02);

  string string03 = "    abc edf g ";
  StringUtil::Trim(TRIM_BOTH, &string03);
  EXPECT_EQ("abc edf g", string03);
}

TEST(StringUtilTest, TestTrim2) {
  string string01 = "   abc edf ";
  EXPECT_EQ("abc edf ", StringUtil::Trim(TRIM_LEFT, string01));

  string string02 = "  abc edf g  ";
  EXPECT_EQ("  abc edf g", StringUtil::Trim(TRIM_RIGHT, string02));

  string string03 = "    abc edf g ";
  EXPECT_EQ("abc edf g", StringUtil::Trim(TRIM_BOTH, string03));
}

TEST(StringUtilTest, TestTrim3) {
  string string01 = "  123 456 ";
  EXPECT_EQ("123 456 ", StringUtil::LTrim(string01));
  EXPECT_EQ("  123 456", StringUtil::RTrim(string01));
  EXPECT_EQ("123 456", StringUtil::Trim(string01));
}

TEST(StringUtilTest, TestDigit2String) {
  EXPECT_EQ("123", StringUtil::Digit2String(123));
  EXPECT_EQ("0.123456", StringUtil::Digit2String(0.123456));
  EXPECT_EQ("-123456", StringUtil::Digit2String(-123456));
  EXPECT_EQ("-123456.120000", StringUtil::Digit2String(-123456.12));
  EXPECT_EQ("-123.100000", StringUtil::Digit2String(-123.1));
}

TEST(StringUtilTest, TestStartWith) {
  EXPECT_TRUE(StringUtil::StartWith("abcd.txt", "ab"));
  EXPECT_TRUE(StringUtil::StartWith("abcd.txt", "abcd.txt"));
  EXPECT_FALSE(StringUtil::StartWith("abcd.txt", "abcd.txt.txt"));
  EXPECT_FALSE(StringUtil::StartWith("", "abc"));
}

TEST(StringUtilTest, TestEndWith) {
  EXPECT_TRUE(StringUtil::EndWith("abc.txt", "txt"));
  EXPECT_TRUE(StringUtil::EndWith("abc.txt", "abc.txt"));
  EXPECT_FALSE(StringUtil::EndWith("abc.txt", "png"));
  EXPECT_FALSE(StringUtil::EndWith("abc.txt", "abc.txt.txt"));
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
