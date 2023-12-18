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

#include "modules/common/configs/config_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/integration_tests/planning_test_base.h"

DECLARE_bool(reckless_change_lane);

namespace apollo {
namespace planning {

/**
 * @class SunnyvaleLoopTest
 * @brief This is an integration test that uses the sunnyvale_loop map.
 */

class SunnyvaleLoopTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "modules/map/data/sunnyvale_loop";
    FLAGS_test_base_map_filename = "base_map_test.bin";
    FLAGS_test_data_dir =
        "modules/planning/planning_base/testdata/sunnyvale_loop_test";
    FLAGS_planning_upper_speed_limit = 12.5;
    FLAGS_use_multi_thread_to_add_obstacles = false;

    FLAGS_enable_rss_info = false;

    ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  }
};

/*
 * test stop for not-nudgable obstacle
 * A cruise test case
 */
TEST_F(SunnyvaleLoopTest, cruise) {
  std::string seq_num = "1";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * stop case to trigger QP ST failed to solve
 */
// TODO(all): need fix test data.
// the existing obstacle is not along reference line and gets ignored
/*
TEST_F(SunnyvaleLoopTest, stop) {
  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}
*/

/*
 * test follow a vehicle with medium distance
 * A follow test case
 */
TEST_F(SunnyvaleLoopTest, follow_01) {
  std::string seq_num = "3";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test nudge a static vehicle with medium distance
 * A nudge test case
 */
TEST_F(SunnyvaleLoopTest, nudge) {
  std::string seq_num = "4";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test follow a vehicle at right turn
 * A follow test case
 */
TEST_F(SunnyvaleLoopTest, follow_02) {
  std::string seq_num = "5";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test follow a vehicle at right turn with a close leading vehicle
 * A follow test case
 */
TEST_F(SunnyvaleLoopTest, follow_03) {
  std::string seq_num = "6";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test slowdown when dp_st_graph is failed.
 * A slowdown test case
 */
TEST_F(SunnyvaleLoopTest, slowdown_01) {
  std::string seq_num = "7";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test right turn.
 * A right turn test case
 */
TEST_F(SunnyvaleLoopTest, rightturn_01) {
  std::string seq_num = "8";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, true);
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test right turn, but stop before traffic light
 * A right turn test case
 * A traffic light test case
 */
TEST_F(SunnyvaleLoopTest, rightturn_with_red_light) {
  std::string seq_num = "8";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_traffic_light_file = seq_num + "_traffic_light.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test change lane
 * A change lane test case
 */
TEST_F(SunnyvaleLoopTest, change_lane) {
  std::string seq_num = "9";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test mission complete
 */
TEST_F(SunnyvaleLoopTest, mission_complete) {
  std::string seq_num = "10";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST(0);
}

/*
 * test change lane with obstacle at target lane
 */
TEST_F(SunnyvaleLoopTest, avoid_change_left) {
  std::string seq_num = "11";
  FLAGS_reckless_change_lane = true;
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test qp path failure
 */
TEST_F(SunnyvaleLoopTest, qp_path_failure) {
  std::string seq_num = "12";
  FLAGS_reckless_change_lane = true;
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * test change lane faillback
 * ADC position passed the change lane zone, failed to change to the new lane
 * and reroute is triggered but new rerouting result is not received yet.
 * Expect to keep going on the current lane.
 */
TEST_F(SunnyvaleLoopTest, change_lane_failback) {
  // temporarily disable this test case, because a lane in routing cannot be
  // found on test map.
  auto target_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
      hdmap::MakeMapId("2020_1_-2"));
  if (target_lane == nullptr) {
    AERROR << "Could not find lane 2020_1_-2 on map " << hdmap::BaseMapFile();
    return;
  }

  std::string seq_num = "13";
  FLAGS_reckless_change_lane = true;
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

}  // namespace planning
}  // namespace apollo

TMAIN;
