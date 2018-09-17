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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "cybertron/common/log.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/file.h"

#include "modules/planning/planning_component.h"

namespace apollo {
namespace planning {

#define RUN_GOLDEN_TEST(sub_case_num)                            \
  {                                                              \
    const ::testing::TestInfo* const test_info =                 \
        ::testing::UnitTest::GetInstance()->current_test_info(); \
    bool no_trajectory_point = false;                            \
    bool run_integration_test_success = RunIntegrationTest(      \
        test_info->name(), sub_case_num, no_trajectory_point);   \
    EXPECT_TRUE(run_integration_test_success);                   \
  }

#define RUN_GOLDEN_TEST_DECISION(sub_case_num)                   \
  {                                                              \
    const ::testing::TestInfo* const test_info =                 \
        ::testing::UnitTest::GetInstance()->current_test_info(); \
    bool no_trajectory_point = true;                             \
    bool run_integration_test_success = RunIntegrationTest(      \
        test_info->name(), sub_case_num, no_trajectory_point);   \
    EXPECT_TRUE(run_integration_test_success);                   \
  }

#define TMAIN                                            \
  int main(int argc, char** argv) {                      \
    ::testing::InitGoogleTest(&argc, argv);              \
    ::google::ParseCommandLineFlags(&argc, &argv, true); \
    return RUN_ALL_TESTS();                              \
  }

DECLARE_string(test_routing_response_file);
DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);
DECLARE_string(test_data_dir);
DECLARE_string(test_prediction_file);
DECLARE_string(test_traffic_light_file);
DECLARE_string(test_relative_map_file);
DECLARE_string(test_previous_planning_file);

class IntegrationTestBase : public ::testing::Test {
 public:
  virtual void SetUp();

  bool RunIntegrationTest(const std::string& test_case_name, int case_num,
                          bool no_trajectory_point);

 protected:
  static void SetUpTestCase();
  bool FeedTestData();
  void UpdateData();
  void PublishAllTopics();

 protected:
  std::unique_ptr<PlanningComponent> planning_;

  // input data
  apollo::canbus::Chassis chassis_;
  apollo::localization::LocalizationEstimate localization_;
  apollo::prediction::PredictionObstacles prediction_;
  apollo::routing::RoutingResponse routing_response_;
  apollo::perception::TrafficLightDetection traffic_light_detection_;

  std::shared_ptr<apollo::cybertron::Node> node_;
};

}  // namespace planning
}  // namespace apollo
