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

#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/planning.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

DECLARE_string(test_routing_response_file);
DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);

/**
 * @class GarageTest
 * @brief This is an integration test that uses the garage map.
 */

class GarageTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "modules/planning/testdata/garage_map";
    FLAGS_base_map_filename = "base_map.txt";
    FLAGS_test_data_dir = "modules/planning/testdata/garage_test";
    FLAGS_planning_upper_speed_limit = 12.5;
    FLAGS_test_routing_response_file = "garage_routing.pb.txt";
    FLAGS_enable_lag_prediction = false;
  }
};

/*
 * test stop for not-nudgable obstacle
 */
TEST_F(GarageTest, stop_obstacle) {
  FLAGS_test_prediction_file = "stop_obstacle_prediction.pb.txt";
  FLAGS_test_localization_file = "stop_obstacle_localization.pb.txt";
  FLAGS_test_chassis_file = "stop_obstacle_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test follow head_vehicle
 */
TEST_F(GarageTest, follow) {
  FLAGS_test_prediction_file = "follow_prediction.pb.txt";
  FLAGS_test_localization_file = "follow_localization.pb.txt";
  FLAGS_test_chassis_file = "follow_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test stop for destination
 */
TEST_F(GarageTest, stop_dest) {
  FLAGS_test_prediction_file = "stop_dest_prediction.pb.txt";
  FLAGS_test_localization_file = "stop_dest_localization.pb.txt";
  FLAGS_test_chassis_file = "stop_dest_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test stop for out of map
 * planning should fail in this case, but the module should not core.
 */
TEST_F(GarageTest, out_of_map) {
  FLAGS_test_prediction_file = "out_of_map_prediction.pb.txt";
  FLAGS_test_localization_file = "out_of_map_localization.pb.txt";
  FLAGS_test_chassis_file = "out_of_map_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test stop passed stop line
 */
TEST_F(GarageTest, stop_over_line) {
  std::string seq_num = "1";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

}  // namespace planning
}  // namespace apollo

TMAIN;
