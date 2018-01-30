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
#include "modules/common/util/dropbox.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/planning.h"
#include "modules/planning/tasks/traffic_decider/stop_sign.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using apollo::common::util::Dropbox;
using apollo::planning::StopSign;

/**
 * @class SunnyvaleBigLoopTest
 * @brief This is an integration test that uses the sunnyvale_big_loop map.
 */

class SunnyvaleBigLoopTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_map_dir = "modules/map/data/sunnyvale_big_loop";
    FLAGS_test_base_map_filename = "base_map.bin";
    FLAGS_test_data_dir = "modules/planning/testdata/sunnyvale_big_loop_test";
    FLAGS_planning_upper_speed_limit = 12.5;
  }
};

/*
 * stop_sign: adc proceed
 *   adc status: null => TO_STOP
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_01) {
  FLAGS_enable_stop_sign = true;
  std::string seq_num = "1";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check dropbox value
  StopSign::StopSignStopStatus* status =
      Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status =
      (status == nullptr) ? StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::TO_STOP, stop_status);
}

/*
 * stop_sign: adc stopped (speed and distance to stop_line)
 *   adc status: TO_STOP => STOPPING
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_02) {
  FLAGS_enable_stop_sign = true;

  // set dropbox
  Dropbox<StopSign::StopSignStopStatus>::Open()->Set(
      "kStopSignStopStatus_1017", StopSign::StopSignStopStatus::TO_STOP);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check dropbox value
  StopSign::StopSignStopStatus* status =
      Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status =
      (status == nullptr) ? StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::STOPPING, stop_status);
}

/*
 * stop_sign: adc stopped + wait_time < 3sec
 *   adc status: STOPPING => STOPPING
 *   decision: STOP
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_03) {
  FLAGS_enable_stop_sign = true;

  // set dropbox
  Dropbox<StopSign::StopSignStopStatus>::Open()->Set(
      "kStopSignStopStatus_1017", StopSign::StopSignStopStatus::STOPPING);
  double stop_start_time = Clock::NowInSeconds() - 2;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_1017", stop_start_time);
  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check dropbox value
  StopSign::StopSignStopStatus* status =
      Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status =
      (status == nullptr) ? StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::STOPPING, stop_status);
}

/*
 * stop_sign: adc stopped + wait time > 3
 *   adc status: STOPPING => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_04) {
  FLAGS_enable_stop_sign = true;

  // set dropbox
  Dropbox<StopSign::StopSignStopStatus>::Open()->Set(
      "kStopSignStopStatus_1017", StopSign::StopSignStopStatus::STOPPING);
  double stop_start_time = Clock::NowInSeconds() - 4;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_1017", stop_start_time);
  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // check dropbox value
  StopSign::StopSignStopStatus* status =
      Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status =
      (status == nullptr) ? StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::STOP_DONE, stop_status);
}

/*
 * stop_sign:
 * bag: 2018-01-24-11-32-28/2018-01-24-11-32-30_0.bag
 * step 1:
 *   adc decision: STOP
 * step 2:
 *   wait_time = 4, other vehicles arrived at other stop sign later than adc
 *   adc status: STOPPING => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_05) {
  FLAGS_enable_stop_sign = true;

  std::string seq_num = "3";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);

  // set dropbox
  double stop_start_time = Clock::NowInSeconds() - 4;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_9762", stop_start_time);

  seq_num = "4";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(1);
}

/*
 * stop_sign:
 * bag: 2018-01-24-11-36-55/2018-01-24-11-36-55
 * step 1:
 *   adc decision: STOP
 * step 2:
 *   wait_time = 4, other vehicles arrived at other stop sign earlier than adc
 *   adc status: STOPPING => STOPPING (i.e. waiting)
 *   decision: STOP
 * step 3:
 *   wait_time = 4,
 *     and other vehicles arrived at other stop sign earlier than adc GONE
 *   adc status: STOPPING => STOPPING => STOP_DONE
 *   decision: CRUISE
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_06) {
  FLAGS_enable_stop_sign = true;

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

  // set dropbox
  double stop_start_time = Clock::NowInSeconds() - 4;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_1022", stop_start_time);

  seq_num = "6";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(1);

  // check dropbox value on watch vehicles
  // waiting for vehicle 4059 on lane 868_1_-1
  std::string db_key_watch_vehicle = "kStopSignWatchVehicle_868_1_-1";
  std::vector<std::string>* value =
      Dropbox<std::vector<std::string>>::Open()->Get(db_key_watch_vehicle);
  EXPECT_TRUE(value != nullptr && (*value)[0] == "4059");

  // step 3:
  // wait time is enough
  // previously watch vehicles are gone

  // set dropbox
  stop_start_time = Clock::NowInSeconds() - 4;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_1022", stop_start_time);

  seq_num = "7";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(2);
}

}  // namespace planning
}  // namespace apollo

TMAIN;
