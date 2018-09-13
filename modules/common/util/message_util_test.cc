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

#include "modules/common/util/message_util.h"
#include "modules/common/util/testdata/simple.pb.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

TEST(StringUtilTest, dump_message) {
  auto a = 1;
  EXPECT_TRUE(DumpMessage(a));
  auto simple_msg = std::make_shared<test::SimpleMessage>();
  FillHeader("test", simple_msg.get());
  EXPECT_TRUE(DumpMessage(simple_msg));
  EXPECT_TRUE(
      PathExists("/tmp/apollo.common.util.test.SimpleMessage/0.pb.txt"));
}

// TEST(StringUtilTest, get_desy_sec) {
//   auto simple_msg = std::make_shared<test::SimpleMessage>();
//   FillHeader("test", simple_msg.get());
//   EXPECT_GT(GetDelaySec(simple_msg), 0);
// }

}  // namespace util
}  // namespace common
}  // namespace apollo
