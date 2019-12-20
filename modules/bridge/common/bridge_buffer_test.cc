/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/bridge/common/bridge_buffer.h"

#include <cstdio>
#include <string>
#include "gtest/gtest.h"

namespace apollo {
namespace bridge {

TEST(BridgeBufferTest, bridge_buf_test) {
  BridgeBuffer<char> buf;
  char *p = buf;
  EXPECT_EQ(0, buf.capacity());
  EXPECT_EQ(0, buf.size());

  buf.reset(100);
  char *p1 = buf;
  EXPECT_EQ(100, buf.capacity());
  EXPECT_EQ(100, buf.size());
  EXPECT_NE(p, p1);

  std::string str("hello world");
  snprintf(buf, str.length() + 1, "%s", str.c_str());
  EXPECT_STREQ(buf, str.c_str());

  buf.reset(80);
  char *p2 = buf;
  EXPECT_EQ(100, buf.capacity());
  EXPECT_EQ(80, buf.size());
  EXPECT_EQ(p2, p1);

  std::string str1("hi world");
  snprintf(buf, str1.length() + 1, "%s", str1.c_str());
  EXPECT_STREQ(buf, str1.c_str());
}

}  // namespace bridge
}  // namespace apollo
