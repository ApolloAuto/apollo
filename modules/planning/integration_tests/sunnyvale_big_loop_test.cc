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
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

/**
 * @class SunnyvaleBigLoopTest
 * @brief This is an integration test that uses the sunnyvale_big_loop map.
 *
 * sequence number allocation:
 *     0 -  99: stop sign
 *   100 - 199: keep clear
 *   200 - 299: crosswalk
 *   300 - 399: signal light
 *   400 - 499: change lane
 *   500 - 599: front vehicle
 *   600 - 699: destination
 */

class SunnyvaleBigLoopTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "modules/map/data/sunnyvale_big_loop";
    FLAGS_test_base_map_filename = "base_map.bin";
    FLAGS_test_data_dir = "modules/planning/testdata/sunnyvale_big_loop_test";
    FLAGS_planning_upper_speed_limit = 12.5;

    FLAGS_enable_scenario_pull_over = false;
    FLAGS_enable_scenario_stop_sign = false;
    FLAGS_enable_scenario_traffic_light = false;
    FLAGS_enable_rss_info = false;

    ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
    ENABLE_RULE(TrafficRuleConfig::DESTINATION, false);
    ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
    ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, false);
  }
};

/*
 * stop_sign:
 *   desc: adc proceed, 27m from stop sign, not enter stop-sign scenario yet,
 *         but the stop decision for stop-sign shall be there
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_01) {
  FLAGS_enable_scenario_stop_sign = true;

  std::string seq_num = "1";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningContext content
  const auto& stop_sign_status =
      PlanningContext::Instance()->planning_status().stop_sign();
  EXPECT_EQ(stop_sign_status.current_stop_sign_overlap_id(), "");
  EXPECT_EQ(stop_sign_status.done_stop_sign_overlap_id(), "");
  EXPECT_EQ(stop_sign_status.wait_for_obstacle_id_size(), 0);
}

/*
 * stop_sign:
 *   desc: adc close enough to stop-sign, enter PRE-STOP stage
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_02) {
  FLAGS_enable_scenario_stop_sign = true;

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningContext content
  const auto& stop_sign_status =
      PlanningContext::Instance()->planning_status().stop_sign();
  EXPECT_EQ(stop_sign_status.current_stop_sign_overlap_id(), "1017");
  EXPECT_EQ(stop_sign_status.done_stop_sign_overlap_id(), "");
  EXPECT_EQ(stop_sign_status.wait_for_obstacle_id_size(), 0);
}

/*
 * stop_sign:
 *   desc: adc stopped + wait_time < STOP_DURATION, stage PRE-STOP => STOP
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_03) {
  FLAGS_enable_scenario_stop_sign = true;

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  // PRE-STOP stage
  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningContext content
  const auto& stop_sign_status =
      PlanningContext::Instance()->planning_status().stop_sign();
  EXPECT_EQ(stop_sign_status.current_stop_sign_overlap_id(), "1017");
  EXPECT_EQ(stop_sign_status.done_stop_sign_overlap_id(), "");
  EXPECT_EQ(stop_sign_status.wait_for_obstacle_id_size(), 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // STOP stage
  RUN_GOLDEN_TEST_DECISION(1);

  // check PlanningContext content
  const auto& stop_sign_status_2 =
      PlanningContext::Instance()->planning_status().stop_sign();
  EXPECT_EQ(stop_sign_status_2.current_stop_sign_overlap_id(), "1017");
  EXPECT_EQ(stop_sign_status_2.done_stop_sign_overlap_id(), "");
  EXPECT_EQ(stop_sign_status_2.wait_for_obstacle_id_size(), 0);
}

/*
 * keep_clear: keep clear zone clear
 * bag: 2018-05-22-13-59-27/2018-05-22-14-09-29_10.bag
 * decision: not stopped by KEEP_CLEAR
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_01) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, false);

  std::string seq_num = "101";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * keep_clear: vehicle inside KEEP Clear zone, with speed and BLOCKING
 * bag: 2018-05-22-13-59-27/2018-05-22-14-13-29_14.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_02) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, false);

  std::string seq_num = "102";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * keep_clear: vehicle inside KEEP Clear zone, with speed and NOT BLOCKING
 * bag: 2018-05-22-13-59-27/2018-05-22-14-13-29_14.bag
 * decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_03) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, false);

  std::string seq_num = "103";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * crosswalk: pedestrian on crosswalk
 * bag: 2018-01-29-17-22-46/2018-01-29-17-31-47_9.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, crosswalk_01) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, true);

  std::string seq_num = "200";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * crosswalk: timeout on static pedestrian on crosswalk
 * bag: 2018-01-29-17-22-46/2018-01-29-17-31-47_9.bag
 * decision: STOP first, and then CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, crosswalk_02) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, true);

  std::string seq_num = "201";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value
  auto* crosswalk_status = PlanningContext::Instance()
                               ->mutable_planning_status()
                               ->mutable_crosswalk();
  EXPECT_EQ("2832", crosswalk_status->crosswalk_id());
  EXPECT_EQ(1, crosswalk_status->stop_time_size());
  EXPECT_EQ("11652", crosswalk_status->stop_time(0).obstacle_id());

  // step 2:
  // timeout on static pedestrian

  // set PlanningStatus
  auto* crosswalk_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::CROSSWALK);
  double stop_timeout = crosswalk_config->crosswalk().stop_timeout();
  double wait_time = stop_timeout + 0.5;
  for (auto& stop_time : *crosswalk_status->mutable_stop_time()) {
    if (stop_time.obstacle_id() == "11652") {
      stop_time.set_obstacle_stop_timestamp(Clock::NowInSeconds() - wait_time);
    }
  }

  RUN_GOLDEN_TEST_DECISION(1);
}

TEST_F(SunnyvaleBigLoopTest, traffic_light_green) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, true);

  std::string seq_num = "300";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_traffic_light_file = seq_num + "_traffic_light.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

TEST_F(SunnyvaleBigLoopTest, change_lane_abort_for_fast_back_vehicle) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, true);

  std::string seq_num = "400";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * destination: stop on arriving destination when pull-over is disabled
 * bag: 2018-05-16-10-00-32/2018-05-16-10-00-32_10.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, destination_stop_01) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::DESTINATION, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::TRAFFIC_LIGHT, false);

  std::string seq_num = "600";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

}  // namespace planning
}  // namespace apollo

TMAIN;
