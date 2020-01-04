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

#include "cyber/common/environment.h"

#include <cstdlib>

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace common {

TEST(EnvironmentTest, get_env) {
  unsetenv("EnvironmentTest_get_env");
  std::string env_value = GetEnv("EnvironmentTest_get_env");
  EXPECT_EQ(env_value, "");
  setenv("EnvironmentTest_get_env", "123456789", 1);
  env_value = GetEnv("EnvironmentTest_get_env");
  EXPECT_EQ(env_value, "123456789");
  unsetenv("EnvironmentTest_get_env");

  const std::string default_value = "DEFAULT_FOR_TEST";
  EXPECT_EQ(default_value, GetEnv("SOMETHING_NOT_EXIST", default_value));
}

TEST(EnvironmentTest, work_root) {
  std::string before = WorkRoot();
  unsetenv("CYBER_PATH");
  std::string after = GetEnv("CYBER_PATH");
  EXPECT_EQ(after, "");
  setenv("CYBER_PATH", before.c_str(), 1);
  after = WorkRoot();
  EXPECT_EQ(after, before);
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
