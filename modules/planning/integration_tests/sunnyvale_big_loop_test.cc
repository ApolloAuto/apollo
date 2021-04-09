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
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/toolkits/deciders/stop_sign.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::time::Clock;
using apollo::planning::StopSign;

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

    ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
    ENABLE_RULE(TrafficRuleConfig::DESTINATION, false);
    ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
    ENABLE_RULE(TrafficRuleConfig::PULL_OVER, false);
    ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
    ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);
  }
};

/*
 * stop_sign: adc proceed
 *   adc status: null => DRIVE
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

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value: DRIVE
  auto stop_sign_status = GetPlanningStatus()->stop_sign();
  EXPECT_TRUE(stop_sign_status.has_status() &&
              stop_sign_status.status() == StopSignStatus::DRIVE);
}

/*
 * stop_sign: adc stopped (speed and distance to stop_line)
 *   adc status: DRIVE => STOP
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_02) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  // set PlanningStatus: stop_status = DRIVE
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_stop_sign_id("1017");
  stop_sign_status->set_status(StopSignStatus::DRIVE);

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value: STOP
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOP);
}

/*
 * stop_sign: adc stopped + wait_time < STOP_DURATION
 *   adc status: STOP => STOP
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_03) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  // set PlanningStatus: wait_time < STOP_DURATION
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_stop_sign_id("1017");
  stop_sign_status->set_status(StopSignStatus::STOP);
  auto* stop_sign_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::STOP_SIGN);
  double stop_duration = stop_sign_config->stop_sign().stop_duration();
  double wait_time = stop_duration - 0.5;
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value: STOP
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOP);
}

/*
 * stop_sign: adc stopped + wait time > STOP_DURATION
 *   adc status: STOP => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_04) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);

  // get config
  auto* stop_sign_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::STOP_SIGN);
  stop_sign_config->mutable_stop_sign()->mutable_creep()->set_enabled(false);

  // update clock
  double stop_duration = stop_sign_config->stop_sign().stop_duration();
  // add 0.5 seconds as buffer time.
  std::chrono::duration<double> time_sec(Clock::NowInSeconds() + stop_duration +
                                         0.5);
  Clock::SetNow(std::chrono::duration_cast<std::chrono::nanoseconds>(time_sec));

  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_stop_sign_id("1017");
  stop_sign_status->set_status(StopSignStatus::STOP);

  RUN_GOLDEN_TEST_DECISION(1);

  // check PlanningStatus value: STOP_DONE
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOP_DONE);
}

/*
 * stop_sign:
 * bag: 2018-01-24-11-32-28/2018-01-24-11-32-30_0.bag
 * step 1:
 *   adc decision: STOP
 * step 2:
 *   wait_time > STOP_DURATION
 *      other vehicles arrived at other stop sign later than adc
 *   adc status: STOP => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_05) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "3";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  // set configs
  auto* stop_sign_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::STOP_SIGN);
  stop_sign_config->mutable_stop_sign()->mutable_creep()->set_enabled(false);

  RUN_GOLDEN_TEST_DECISION(0);

  // step 2

  seq_num = "4";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::UpdateData();

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  double stop_duration = stop_sign_config->stop_sign().stop_duration();
  double wait_time = stop_duration + 1;
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  RUN_GOLDEN_TEST_DECISION(1);
}

/*
 * stop_sign:
 * bag: 2018-01-24-11-36-55/2018-01-24-11-36-57_0.bag
 * step 1:
 *   adc status: DRIVE
 *   adc decision: STOP
 * step 2:
 *   wait_time > STOP_DURATION,
 *      other vehicles arrived at other stop sign earlier than adc
 *   adc status: STOP => WAIT (i.e. waiting)
 *   decision: STOP
 * step 3:
 *   wait_time > STOP_DURATION,
 *     and other vehicles arrived at other stop sign earlier than adc GONE
 *   adc status: STOP => WAIT (with watch vehicle -> empty)
 *   decision: STOP
 * step 4:
 *   adc status: WAIT => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_06) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "5";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  // set config
  auto* stop_sign_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::STOP_SIGN);
  stop_sign_config->mutable_stop_sign()->mutable_creep()->set_enabled(false);

  RUN_GOLDEN_TEST_DECISION(0);

  // step 2:
  // wait time is enough
  // but vehicles are still there (use the same data as previous test)

  seq_num = "6";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::UpdateData();

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_status(StopSignStatus::STOP);
  double stop_duration = stop_sign_config->stop_sign().stop_duration();
  double wait_time = stop_duration + 0.5;
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  RUN_GOLDEN_TEST_DECISION(1);

  // check PlanningStatus value: WAIT
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::WAIT);
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

  seq_num = "7";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::UpdateData();

  // set PlanningStatus
  stop_sign_status->set_status(StopSignStatus::WAIT);
  stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  RUN_GOLDEN_TEST_DECISION(2);

  // check PlanningStatus value: WAIT
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::WAIT);
  // check PlanningStatus value on watch vehicles
  EXPECT_EQ(0, stop_sign_status->lane_watch_vehicles_size());

  // step 4:
  // WAIT(watched vehicle is empty) => STOP_DONE

  RUN_GOLDEN_TEST_DECISION(3);
  // check PlanningStatus value: STOP_DONE
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOP_DONE);
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

  // set config
  auto* stop_sign_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::STOP_SIGN);
  stop_sign_config->mutable_stop_sign()->mutable_creep()->set_enabled(false);

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value: DRIVE
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  EXPECT_EQ("9762", stop_sign_status->stop_sign_id());
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::DRIVE);
  EXPECT_FALSE(stop_sign_status->has_stop_start_time());
  // waiting for vehicle 4059 on lane 868_1_-1
  EXPECT_EQ(1, stop_sign_status->lane_watch_vehicles_size());
  auto lane_watch_vehicles = stop_sign_status->lane_watch_vehicles(0);
  EXPECT_EQ("1706a_1_-1", lane_watch_vehicles.lane_id());
  EXPECT_TRUE(lane_watch_vehicles.watch_vehicles_size() == 1 &&
              lane_watch_vehicles.watch_vehicles(0) == "12257");

  // step 2: pass stop sign
  seq_num = "13";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::UpdateData();

  RUN_GOLDEN_TEST_DECISION(1);

  // check PlanningStatus value: clear
  // to make sure everything is cleared for that stop sign
  EXPECT_FALSE(GetPlanningStatus()->has_stop_sign());

  // step 3: 2nd round

  seq_num = "12";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(2);
}

/*
 * stop_sign:
 * bag: 2018-05-16-10-00-32/2018-05-16-10-00-32_10.bag
 * step 1:
 *   adc status: STOP
 *   adc decision: STOP
 * step 2:
 *   wait_time > STOP_DURATION,
 *   adc status: STOP => CREEP
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_08) {
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, true);

  std::string seq_num = "14";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  // set config
  auto* stop_sign_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::STOP_SIGN);
  stop_sign_config->mutable_stop_sign()->mutable_creep()->set_enabled(true);

  RUN_GOLDEN_TEST_DECISION(0);

  // step 2:
  // wait time is enough
  // but vehicles are still there (use the same data as previous test)

  // set PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->set_status(StopSignStatus::STOP);
  double stop_duration = stop_sign_config->stop_sign().stop_duration();
  double wait_time = stop_duration + 0.5;
  double stop_start_time = Clock::NowInSeconds() - wait_time;
  stop_sign_status->set_stop_start_time(stop_start_time);

  // set config
  stop_sign_config->mutable_stop_sign()
      ->mutable_creep()
      ->set_creep_distance_to_stop_line(1.0);
  stop_sign_config->mutable_stop_sign()
      ->mutable_creep()
      ->set_max_valid_stop_distance(1.0);

  RUN_GOLDEN_TEST_DECISION(1);

  // check PlanningStatus value: CREEP
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::CREEP);

  // step 3: STOP_DONE

  // set config: to make it s valid stop for the same data file
  stop_sign_config->mutable_stop_sign()
      ->mutable_creep()
      ->set_max_valid_stop_distance(4.0);

  RUN_GOLDEN_TEST_DECISION(2);

  // check PlanningStatus value
  EXPECT_TRUE(stop_sign_status->has_status() &&
              stop_sign_status->status() == StopSignStatus::STOP_DONE);
}

/*
 * kee_clear: keep clear zone clear
 * bag: 2018-05-22-13-59-27/2018-05-22-14-09-29_10.bag
 * decision: not stopped by KEEP_CLEAR
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_01) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

  std::string seq_num = "101";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * kee_clear: vehicle inside KEEP Clear zone, with speed and BLOCKING
 * bag: 2018-05-22-13-59-27/2018-05-22-14-13-29_14.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_02) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

  std::string seq_num = "102";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * kee_clear: vehicle inside KEEP Clear zone, with speed and NOT BLOCKING
 * bag: 2018-05-22-13-59-27/2018-05-22-14-13-29_14.bag
 * decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_03) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

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
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

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
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

  std::string seq_num = "201";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value
  auto* crosswalk_status = GetPlanningStatus()->mutable_crosswalk();
  EXPECT_EQ("2832", crosswalk_status->crosswalk_id());
  EXPECT_EQ(1, crosswalk_status->stop_timers_size());
  EXPECT_EQ("11652", crosswalk_status->stop_timers(0).obstacle_id());

  // step 2:
  // timeout on static pesestrian

  // set PlanningStatus
  auto* crosswalk_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::CROSSWALK);
  double stop_timeout = crosswalk_config->crosswalk().stop_timeout();
  double wait_time = stop_timeout + 0.5;
  for (int i = 0; i < crosswalk_status->stop_timers_size(); ++i) {
    auto stop_timer = crosswalk_status->mutable_stop_timers(i);
    if (stop_timer->obstacle_id() == "11652") {
      stop_timer->set_stop_time(Clock::NowInSeconds() - wait_time);
    }
  }

  RUN_GOLDEN_TEST_DECISION(1);
}

TEST_F(SunnyvaleBigLoopTest, traffic_light_green) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, true);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

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
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, true);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

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
  ENABLE_RULE(TrafficRuleConfig::PULL_OVER, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

  std::string seq_num = "600";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();

  // set config
  auto* destination_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::DESTINATION);
  destination_config->mutable_destination()->set_enable_pull_over(false);

  RUN_GOLDEN_TEST_DECISION(0);
}

/*
 * destination: pull-over on arriving destination
 * bag: 2018-05-16-10-00-32/2018-05-16-10-00-32_10.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, destination_pull_over_01) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::DESTINATION, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::PULL_OVER, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

  std::string seq_num = "601";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();

  // set config
  auto* destination_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::DESTINATION);
  destination_config->mutable_destination()->set_enable_pull_over(true);
  destination_config->mutable_destination()->set_pull_over_plan_distance(35.0);

  auto* pull_over_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::PULL_OVER);
  pull_over_config->mutable_pull_over()->set_plan_distance(35.0);
  pull_over_config->mutable_pull_over()->set_operation_length(10.0);

  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value: PULL OVER
  auto* planning_status = GetPlanningStatus();
  EXPECT_TRUE(planning_status->has_pull_over() &&
              planning_status->pull_over().in_pull_over());
  EXPECT_EQ(PullOverStatus::DESTINATION, planning_status->pull_over().reason());

  common::PointENU start_point_0 = planning_status->pull_over().start_point();
  common::PointENU stop_point_0 = planning_status->pull_over().stop_point();
  double stop_point_heading_0 =
      planning_status->pull_over().stop_point_heading();
  double status_set_time_0 = planning_status->pull_over().status_set_time();

  // check PULL OVER decision
  RUN_GOLDEN_TEST_DECISION(1);

  EXPECT_TRUE(planning_status->has_pull_over() &&
              planning_status->pull_over().in_pull_over());
  EXPECT_EQ(PullOverStatus::DESTINATION, planning_status->pull_over().reason());

  common::PointENU start_point_1 = planning_status->pull_over().start_point();
  common::PointENU stop_point_1 = planning_status->pull_over().stop_point();
  double stop_point_heading_1 =
      planning_status->pull_over().stop_point_heading();
  double status_set_time_1 = planning_status->pull_over().status_set_time();

  // start_point/stop_point/etc shall be the same among cycles
  EXPECT_DOUBLE_EQ(start_point_0.x(), start_point_1.x());
  EXPECT_DOUBLE_EQ(start_point_0.y(), start_point_1.y());
  EXPECT_DOUBLE_EQ(stop_point_0.x(), stop_point_1.x());
  EXPECT_DOUBLE_EQ(stop_point_0.y(), stop_point_1.y());
  EXPECT_DOUBLE_EQ(stop_point_heading_0, stop_point_heading_1);
  EXPECT_DOUBLE_EQ(status_set_time_0, status_set_time_1);
}

/*
 * destination: stop inlane while pull over fails
 * bag: 2018-05-16-10-00-32/2018-05-16-10-00-32_10.bag
 * decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, destination_pull_over_02) {
  ENABLE_RULE(TrafficRuleConfig::CROSSWALK, false);
  ENABLE_RULE(TrafficRuleConfig::DESTINATION, true);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);
  ENABLE_RULE(TrafficRuleConfig::PULL_OVER, true);
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::STOP_SIGN, false);

  std::string seq_num = "601";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();

  // set config
  auto* destination_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::DESTINATION);
  destination_config->mutable_destination()->set_enable_pull_over(true);
  destination_config->mutable_destination()->set_pull_over_plan_distance(20.0);

  auto* pull_over_config =
      PlanningTestBase::GetTrafficRuleConfig(TrafficRuleConfig::PULL_OVER);
  pull_over_config->mutable_pull_over()->set_plan_distance(20.0);
  pull_over_config->mutable_pull_over()->set_operation_length(5.0);

  // step 1: pull over
  RUN_GOLDEN_TEST_DECISION(0);

  // check PlanningStatus value: PULL OVER
  auto* planning_status = GetPlanningStatus();
  EXPECT_TRUE(planning_status->has_pull_over() &&
              planning_status->pull_over().in_pull_over());
  EXPECT_EQ(PullOverStatus::DESTINATION, planning_status->pull_over().reason());

  // step 2: pull over failed, stop inlane

  // clear PlanningStatus
  planning_status->mutable_pull_over()->clear_status();
  // set destination point further to make inlane stop
  // be able to use this dest point stop fence so that the test result is fixed
  // instead of using adc's position which may differ for some reason
  planning_status->mutable_pull_over()->mutable_inlane_dest_point()->set_x(
      587163.741);
  planning_status->mutable_pull_over()->mutable_inlane_dest_point()->set_y(
      4141196.136);

  // set config
  pull_over_config->mutable_pull_over()->set_operation_length(21.0);
  pull_over_config->mutable_pull_over()->set_max_failure_count(1);
  pull_over_config->mutable_pull_over()->set_max_stop_deceleration(3.0);

  // check PULL OVER decision
  RUN_GOLDEN_TEST_DECISION(1);

  // check PlanningStatus value: PULL OVER  cleared
  EXPECT_FALSE(planning_status->has_pull_over());
}

/*
// TODO(all): this test need rewrite
TEST_F(SunnyvaleBigLoopTest, bypass_parked_bus) {
  ENABLE_RULE(TrafficRuleConfig::SIGNAL_LIGHT, false);
  ENABLE_RULE(TrafficRuleConfig::KEEP_CLEAR, false);

  double acc_lower_bound = FLAGS_longitudinal_acceleration_lower_bound;
  FLAGS_longitudinal_acceleration_lower_bound = -5.0;
  std::string seq_num = "500";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  PlanningTestBase::SetUp();

  RUN_GOLDEN_TEST(0);
  FLAGS_longitudinal_acceleration_lower_bound = acc_lower_bound;
}
*/

}  // namespace planning
}  // namespace apollo

TMAIN;
