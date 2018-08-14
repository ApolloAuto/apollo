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

#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"

#define private public
#include "modules/perception/perception.h"

namespace apollo {
namespace perception {

using common::adapter::AdapterManager;

#define RUN_GOLDEN_TEST(sub_case_num)                            \
  {                                                              \
    const ::testing::TestInfo* const test_info =                 \
        ::testing::UnitTest::GetInstance()->current_test_info(); \
    bool run_perception_success =                                \
        RunPerception(test_info->name(), sub_case_num);          \
    EXPECT_TRUE(run_perception_success);                         \
  }

#define TMAIN                                            \
  int main(int argc, char** argv) {                      \
    ::testing::InitGoogleTest(&argc, argv);              \
    ::google::ParseCommandLineFlags(&argc, &argv, true); \
    return RUN_ALL_TESTS();                              \
  }

DECLARE_string(test_data_dir);
DECLARE_string(test_pointcloud_file);
DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);
DECLARE_string(perception_config_file);
DECLARE_string(perception_adapter_config_filename);

class PerceptionTestBase : public ::testing::Test {
 public:
  static void SetUpTestCase();

  virtual void SetUp();

  void UpdateData();

  /**
   * Execute the perception code.
   * @return true if perception is success. The ADCTrajectory will be used to
   * store the planing results.  Otherwise false.
   */
  bool RunPerception(const std::string& test_case_name, int case_num);

 protected:
  bool SetUpAdapters();

  Perception perception_;
};

}  // namespace perception
}  // namespace apollo
