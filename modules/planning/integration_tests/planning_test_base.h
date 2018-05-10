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

#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/planning/proto/dp_poly_path_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"

#define private public
#include "modules/planning/planning.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

#define RUN_GOLDEN_TEST(sub_case_num)                                         \
  {                                                                           \
    const ::testing::TestInfo* const test_info =                              \
        ::testing::UnitTest::GetInstance()->current_test_info();              \
    bool run_planning_success = RunPlanning(test_info->name(), sub_case_num); \
    EXPECT_TRUE(run_planning_success);                                        \
  }

#define TMAIN                                            \
  int main(int argc, char** argv) {                      \
    ::testing::InitGoogleTest(&argc, argv);              \
    ::google::ParseCommandLineFlags(&argc, &argv, true); \
    return RUN_ALL_TESTS();                              \
  }

#define ENABLE_RULE(RULE_ID, ENABLED) this->rule_enabled_[RULE_ID] = ENABLED

DECLARE_string(test_routing_response_file);
DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);
DECLARE_string(test_data_dir);
DECLARE_string(test_prediction_file);
DECLARE_string(test_traffic_light_file);
DECLARE_string(test_relative_map_file);
DECLARE_string(test_previous_planning_file);

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

 protected:
  void TrimPlanning(ADCTrajectory* origin);
  bool SetUpAdapters();
  bool IsValidTrajectory(const ADCTrajectory& trajectory);

  Planning planning_;
  std::map<TrafficRuleConfig::RuleId, bool> rule_enabled_;
  ADCTrajectory adc_trajectory_;
};

}  // namespace planning
}  // namespace apollo
