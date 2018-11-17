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
#include "modules/common/util/string_tokenizer.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

using ::testing::ElementsAre;

TEST(UtilTest, StringTokenizer) {
  std::string str("aa,bbb,c");
  std::string delim(",");

  EXPECT_THAT(StringTokenizer::Split(str, delim),
              ElementsAre("aa", "bbb", "c"));
}

TEST(UtilTest, StringTokenizerNext) {
  std::string str("     aa,  bbb , c   ");
  std::string delim(", ");

  StringTokenizer stn(str, delim);
  auto t0 = stn.Next();
  auto t1 = stn.Next();
  auto t2 = stn.Next();
  auto t3 = stn.Next();
  auto t4 = stn.Next();

  EXPECT_EQ(t0, "aa");
  EXPECT_EQ(t1, "bbb");
  EXPECT_EQ(t2, "c");
  EXPECT_EQ(t3, "");
  EXPECT_EQ(t4, "");
}

TEST(UtilTest, StringTokenizerEmptyString) {
  std::string str("");
  std::string delim(", ");

  StringTokenizer stn(str, delim);
  auto t0 = stn.Next();
  auto t1 = stn.Next();

  EXPECT_EQ(t0, "");
  EXPECT_EQ(t1, "");
}

TEST(UtilTest, StringTokenizerEmptyDelim) {
  std::string str("     aa,  bbb , c   ");
  std::string delim("");

  StringTokenizer stn(str, delim);
  auto t0 = stn.Next();
  auto t1 = stn.Next();

  EXPECT_EQ(t0, str);
  EXPECT_EQ(t1, "");
}

TEST(UtilTest, StringTokenizerBothEmpty) {
  std::string str("");
  std::string delim("");

  StringTokenizer stn(str, delim);
  auto t0 = stn.Next();
  auto t1 = stn.Next();

  EXPECT_EQ(t0, "");
  EXPECT_EQ(t1, "");
}

}  // namespace util
}  // namespace common
}  // namespace apollo
