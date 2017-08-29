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

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/planning/proto/dp_poly_path_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/planning.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

#define RUN_GOLDEN_TEST                                            \
  {                                                                \
    const ::testing::TestInfo* const test_info =                   \
        ::testing::UnitTest::GetInstance()->current_test_info();   \
    bool run_planning_success = RunPlanning(test_info->name(), 0); \
    EXPECT_TRUE(run_planning_success);                             \
  }

DECLARE_string(test_routing_response_file);
DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);
DECLARE_string(test_data_dir);
DECLARE_string(test_prediction_file);

class PlanningTestBase : public ::testing::Test {
 public:
  static void SetUpTestCase();

  virtual void SetUp();

  /**
   * Execute the planning code.
   * @return true if planning is success. The ADCTrajectory will be used to
   * store the planing results.  Otherwise false.
   */
  bool RunPlanning(const std::string& test_case_name, int case_num);

  /**
   * @brief Print out the points to a file for debug and visualization purpose.
   * User can see the file, or feed it
   * into a graphic visualizer.
   */
  static void export_sl_points(
      const std::vector<std::vector<common::SLPoint>>& points,
      const std::string& filename);

  static void export_path_data(const PathData& path_data,
                               const std::string& filename);

 protected:
  void TrimPlanning(ADCTrajectory* origin);
  bool SetUpAdapters();

  Planning planning_;
  ADCTrajectory adc_trajectory_;
};

}  // namespace planning
}  // namespace apollo
