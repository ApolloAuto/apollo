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
#include "modules/common/util/dropbox.h"
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/planning.h"
#include "modules/planning/tasks/traffic_decider/stop_sign.h"

namespace apollo {
namespace planning {

using apollo::common::util::Dropbox;
using apollo::common::time::Clock;
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
 *   decision: stop
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_01) {
  FLAGS_enable_stop_sign = true;
  std::string seq_num = "1";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST;

  // check dropbox value
  StopSign::StopSignStopStatus* status
      = Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status = (status == nullptr) ?
      StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::TO_STOP, stop_status);
}

/*
 * stop_sign: adc stopped (speed and distance to stop_line)
 *   adc status: TO_STOP => STOPPING
 *   decision: stop
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_02) {
  FLAGS_enable_stop_sign = true;

  // set dropbox
  Dropbox<StopSign::StopSignStopStatus>::Open()->Set(
      "kStopSignStopStatus_1017",
      StopSign::StopSignStopStatus::TO_STOP);

  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST;

  // check dropbox value
  StopSign::StopSignStopStatus* status
      = Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status = (status == nullptr) ?
      StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::STOPPING, stop_status);
}

/*
 * stop_sign: adc stopped + wait_time < 3sec
 *   adc status: STOPPING => STOPPING
 *   decision: stop
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_03) {
  FLAGS_enable_stop_sign = true;

  // set dropbox
  Dropbox<StopSign::StopSignStopStatus>::Open()->Set(
      "kStopSignStopStatus_1017",
      StopSign::StopSignStopStatus::STOPPING);
  double stop_start_time = Clock::NowInSeconds() - 2;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_1017",
                               stop_start_time);
  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST;

  // check dropbox value
  StopSign::StopSignStopStatus* status
      = Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status = (status == nullptr) ?
      StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::STOPPING, stop_status);
}

/*
 * stop_sign: adc stopped + wait time > 3
 *   adc status: STOPPING => STOP_DONE
 *   decision: stop
 */
TEST_F(SunnyvaleBigLoopTest, stop_sign_04) {
  FLAGS_enable_stop_sign = true;

  // set dropbox
  Dropbox<StopSign::StopSignStopStatus>::Open()->Set(
      "kStopSignStopStatus_1017",
      StopSign::StopSignStopStatus::STOPPING);
  double stop_start_time = Clock::NowInSeconds() - 4;
  Dropbox<double>::Open()->Set("kStopSignStopStarttime_1017",
                               stop_start_time);
  std::string seq_num = "2";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST;

  // check dropbox value
  StopSign::StopSignStopStatus* status
      = Dropbox<StopSign::StopSignStopStatus>::Open()->Get(
          "kStopSignStopStatus_1017");
  StopSign::StopSignStopStatus stop_status = (status == nullptr) ?
      StopSign::StopSignStopStatus::UNKNOWN : *status;
  EXPECT_EQ(StopSign::StopSignStopStatus::STOP_DONE, stop_status);
}

}  // namespace planning
}  // namespace apollo

TMAIN;
