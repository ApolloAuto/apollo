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

#include "modules/common/configs/cpu_bind_helper.h"

#include <cmath>
#include <string>

#include <unistd.h>
#include <sched.h>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {

class CpuBindHelperTest : public ::testing::Test {
 public:
  void SetUp() {
    //
  }
};

TEST_F(CpuBindHelperTest, Init_BindCpu) {
  CpuBindHelper::instance()->Init("/apollo/modules/common/data/cpu_bind.yaml");
  CpuBindHelper::instance()->BindCpu("cpu_bind_helper_test");

  const std::unordered_map<std::string, std::vector<int>>::const_iterator it =
      CpuBindHelper::instance()->bindrule_map_.find("cpu_bind_helper_test");
  EXPECT_NE(it, CpuBindHelper::instance()->bindrule_map_.end());

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  int ret = sched_getaffinity(0, sizeof(cpuset), &cpuset);
  EXPECT_EQ(0, ret);
  const int core_array[] = {0, 2, 3};
  const std::size_t size = sizeof(core_array) / sizeof(int);
  for (std::size_t i = 0; i < size; ++i) {
    EXPECT_TRUE(CPU_ISSET(core_array[i], &cpuset));
  }
}

}  // namespace common
}  // namespace apollo
