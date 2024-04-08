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

#pragma once

#include <string>

#include "gtest/gtest.h"

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

#include "modules/control/control_component/common/control_gflags.h"
#include "modules/control/control_component/control_component.h"

#define RUN_GOLDEN_TEST                                            \
  {                                                                \
    const ::testing::TestInfo *const test_info =                   \
        ::testing::UnitTest::GetInstance()->current_test_info();   \
    bool run_control_success = test_control(test_info->name(), 0); \
    EXPECT_TRUE(run_control_success);                              \
  }

DECLARE_string(test_localization_file);
DECLARE_string(test_pad_file);
DECLARE_string(test_planning_file);
DECLARE_string(test_chassis_file);
DECLARE_string(test_data_dir);
DECLARE_string(test_monitor_file);

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

class ControlTestBase : public ::testing::Test {
 public:
  static void SetUpTestCase();

  virtual void SetUp();

  bool test_control();
  bool test_control(const std::string &test_case_name, int case_num);
  void LoadControllerPlugin();

 private:
  void trim_control_command(apollo::control::ControlCommand *origin);
  ControlCommand control_command_;
  ControlComponent control_;
  static uint32_t s_seq_num_;
};

}  // namespace control
}  // namespace apollo
