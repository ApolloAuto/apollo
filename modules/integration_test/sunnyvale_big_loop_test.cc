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

#include <string>

#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/time/time.h"
#include "modules/integration_test/integration_test_base.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
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

class SunnyvaleBigLoopTest : public IntegrationTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "modules/map/data/sunnyvale_big_loop";
    FLAGS_test_base_map_filename = "base_map.bin";
    FLAGS_test_data_dir = "modules/planning/testdata/sunnyvale_big_loop_test";
    FLAGS_planning_upper_speed_limit = 12.5;
  }
};

/*
 * kee_clear: keep clear zone clear
 * bag: 2018-05-22-13-59-27/2018-05-22-14-09-29_10.bag
 * decision: not stopped by KEEP_CLEAR
 */
TEST_F(SunnyvaleBigLoopTest, keep_clear_01) {
  std::string seq_num = "101";
  FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
  FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  IntegrationTestBase::SetUp();

  RUN_GOLDEN_TEST_DECISION(0);
}

}  // namespace planning
}  // namespace apollo

TMAIN;
