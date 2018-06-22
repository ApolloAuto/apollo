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

#ifndef MODULES_THIRD_PARTY_PERCEPTION_INTEGRATION_TESTS_H_
#define MODULES_THIRD_PARTY_PERCEPTION_INTEGRATION_TESTS_H_

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/macro.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/proto/radar_obstacle.pb.h"
#include "modules/third_party_perception/third_party_perception.h"

#define RUN_GOLDEN_TEST                                          \
  {                                                              \
    const ::testing::TestInfo *const test_info =                 \
        ::testing::UnitTest::GetInstance()->current_test_info(); \
    bool run_third_party_perception_success =                    \
        test_third_party_perception(test_info->name(), 0);       \
    EXPECT_TRUE(run_third_party_perception_success);             \
  }

DECLARE_string(test_localization_file);

DECLARE_string(test_chassis_file);
DECLARE_string(test_data_dir);
DECLARE_string(test_monitor_file);

/**
 * @namespace apollo::third_party_perception
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {

class ThirdPartyPerceptionTestBase : public ::testing::Test {
 public:
  static void SetUpTestCase();

  virtual void SetUp();

 private:
  static uint32_t s_seq_num_;
};

}  // namespace third_party_perception
}  // namespace apollo

#endif  // MODULES_THIRD_PARTY_PERCEPTION_INTEGRATION_TESTS_H_
