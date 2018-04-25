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
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/planning.h"
#include "modules/planning/tasks/traffic_decider/stop_sign.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using apollo::planning::StopSign;
using apollo::planning::util::GetPlanningStatus;

/**
 * @class SunnyvaleBigLoopTest
 * @brief This is an integration test that uses the sunnyvale_big_loop map.
 */

class SunnyvaleBigLoopTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "modules/map/data/sunnyvale_big_loop";
    FLAGS_test_base_map_filename = "base_map.bin";
    FLAGS_test_data_dir = "modules/planning/testdata/sunnyvale_big_loop_test";
    FLAGS_planning_upper_speed_limit = 12.5;

    ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);
    ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
    ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  }
};

/*
 * stop_sign: adc proceed
 *   adc status: null => TO_STOP
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_01) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);
  std::string seq_num = "1";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check PlanningStatus value
  auto stop_sign_status = GetPlanningStatus()->stop_sign();
  EXPECT_TRUE(stop_sign_status.has_status() &&
              stop_sign_status.status() == StopSignStatus::TO_STOP);
}

/*
 * stop_sign: adc stopped (speed and distance to stop_line)
 *   adc status: TO_STOP => STOPPING
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_02) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_stop_sign_id("1017");
  stop_sign_status->set_status(StopSignStatus::TO_STOP);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check PlanningStatus value
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOPPING);
}

/*
 * stop_sign: adc stopped + wait_time < STOP_DURATION
 *   adc status: STOPPING => STOPPING
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_03) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);
  constexpr double STOP_DURATION = 1;
  double wait_time = STOP_DURATION - 0.5;

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_stop_sign_id("1017");
  stop_sign_status->set_status(StopSignStatus::STOPPING);
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check PlanningStatus value
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOPPING);
}

/*
 * stop_sign: adc stopped + wait time > STOP_DURATION
 *   adc status: STOPPING => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_04) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);
  constexpr double STOP_DURATION = 1;
  double wait_time = STOP_DURATION + 0.5;

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_stop_sign_id("1017");
  stop_sign_status->set_status(StopSignStatus::STOPPING);
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check PlanningStatus value
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOP_DONE);
}

/*
 * stop_sign:
 * bag: 2018-01-24-11-32-28/2018-01-24-11-32-30_0.bag
 * step 1:
 *   adc decision: STOP
 * step 2:
 *   wait_time > stop_duration(1)
 *      other vehicles arrived at other stop sign later than adc
 *   adc status: STOPPING => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_05) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);
  double stop_duration = 1;
  double wait_time = stop_duration + 1;

  std::string seq_num = "3";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  seq_num = "4";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(1);
}

/*
 * stop_sign:
 * bag: 2018-01-24-11-36-55/2018-01-24-11-36-57_0.bag
 * step 1:
 *   adc decision: STOP
 * step 2:
 *   wait_time > stop_duration(1),
 *      other vehicles arrived at other stop sign earlier than adc
 *   adc status: STOPPING => STOPPING (i.e. waiting)
 *   decision: STOP
 * step 3:
 *   wait_time > STOP_DURATION,
 *     and other vehicles arrived at other stop sign earlier than adc GONE
 *   adc status: STOPPING => STOPPING => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_06) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);
  constexpr double STOP_DURATION = 1;
  double wait_time = STOP_DURATION + 0.5;

  std::string seq_num = "5";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // step 2:
  // wait time is enough
  // but vehicles are still there (use the same data as previous test)

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  seq_num = "6";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(1);

  // check PlanningStatus value on watch vehicles
  // waiting for vehicle 4059 on lane 868_1_-1
  EXPECT_EQ(1, stop_sign_status->lane_watch_vehicles_size());
  auto lane_watch_vehicles = stop_sign_status->lane_watch_vehicles(0);
  EXPECT_EQ("868_1_-1", lane_watch_vehicles.lane_id());
  EXPECT_TRUE(lane_watch_vehicles.watch_vehicles_size() == 1 &&
              lane_watch_vehicles.watch_vehicles(0) == "4059");

  // step 3:
  // wait time is enough
  // previously watch vehicles are gone

  // set PlanningStatus
  stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  seq_num = "7";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(2);
}

/*
 * stop_sign:
 * bag:
 *    step 1/3: 22018-02-15-16-37-45/2018-02-15-16-40-46_3.bag
 *    step2:    22018-02-15-16-37-45/2018-02-15-16-41-46_4.bag
 * step 1:
 *   adc decision: STOP
 * step 2:
 *   pass stop sign
 * step 3:
 *   come back to the same stop sign 2nd time
 *   adc decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_07) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "12";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check PlanningStatus value to make sure they are set
  auto stop_sign_status = GetPlanningStatus()->stop_sign();
  EXPECT_EQ("9762", stop_sign_status.stop_sign_id());
  EXPECT_TRUE(stop_sign_status.has_status() &&
              stop_sign_status.status() == StopSignStatus::TO_STOP);
  EXPECT_FALSE(stop_sign_status.has_stop_start_time());

  // step 2: pass stop sign
  seq_num = "13";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(1);

  // check PlanningStatus value
  // to make sure everything is cleared for that stop sign
  EXPECT_FALSE(GetPlanningStatus()->has_stop_sign());

  // step 3: 2nd round
  seq_num = "12";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(2);
}

/*
 * crosswalk: pedestrian on crosswalk
 * bag: 2018-01-29-17-22-46/2018-01-29-17-31-47_9.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, crosswalk_01) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  std::string seq_num = "8";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

/*
 * kee_clear: not blocking, KEEP_CLEAR static obstacle built
 * bag: 2018-01-29-17-22-46/2018-01-29-17-22-47_0.bag
 * decision: CRUISE
 */
/*
TEST_F(SunnyvaleBigLoopTest, keep_clear_01) {
  std::string seq_num = "9";
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}
*/

TEST_F(SunnyvaleBigLoopTest, traffic_light_green) {
  std::string seq_num = "10";
  FLAGS_enable_prediction = false;
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);

  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_traffic_light_file = seq_num + "_traffic_light.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
  FLAGS_enable_prediction = true;
}

TEST_F(SunnyvaleBigLoopTest, abort_change_lane_for_fast_back_vehicle) {
  std::string seq_num = "11";
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);

  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}

TEST_F(SunnyvaleBigLoopTest, bypass_parked_bus) {
  std::string seq_num = "14";
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  double acc_lower_bound = FLAGS_longitudinal_acceleration_lower_bound;
  FLAGS_longitudinal_acceleration_lower_bound = -5.0;

  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
  FLAGS_longitudinal_acceleration_lower_bound = acc_lower_bound;
}

}  // namespace planning
}  // namespace apollo

TMAIN;
