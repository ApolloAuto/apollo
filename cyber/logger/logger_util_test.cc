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

#include "cyber/logger/logger_util.h"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace logger {

TEST(LoggerUtilTest, GetHostName) {
  std::string host_name;
  GetHostName(&host_name);
  EXPECT_FALSE(host_name.empty());
}

TEST(LoggerUtilTest, FindModuleName) {
  std::string module_name = "module_name_0";
  std::string log_message = LEFT_BRACKET;
  FindModuleName(&log_message, &module_name);
  EXPECT_EQ("module_name_0", module_name);

  log_message = RIGHT_BRACKET;
  FindModuleName(&log_message, &module_name);
  EXPECT_EQ("module_name_0", module_name);

  log_message = LEFT_BRACKET;
  log_message.append("module_name_1");
  log_message.append(RIGHT_BRACKET);
  FindModuleName(&log_message, &module_name);
  EXPECT_EQ("module_name_1", module_name);

  log_message = "123";
  log_message.append(LEFT_BRACKET);
  log_message.append("module_name_2");
  log_message.append(RIGHT_BRACKET);
  log_message.append("123");
  FindModuleName(&log_message, &module_name);
  EXPECT_EQ("module_name_2", module_name);

  log_message = "123";
  log_message.append(LEFT_BRACKET);
  log_message.append(RIGHT_BRACKET);
  log_message.append("123");
  FindModuleName(&log_message, &module_name);
  EXPECT_EQ("logger_util_test_" + std::to_string(GetMainThreadPid()),
            module_name);
}

TEST(LoggerUtilTest, MaxLogSize) {
  FLAGS_max_log_size = 10;
  EXPECT_EQ(10, MaxLogSize());
  FLAGS_max_log_size = 0;
  EXPECT_EQ(1, MaxLogSize());
}

TEST(LoggerUtilTest, PidHasChanged) { EXPECT_FALSE(PidHasChanged()); }

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
