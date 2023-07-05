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
