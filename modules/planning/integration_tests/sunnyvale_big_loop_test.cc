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
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/planning.h"

DECLARE_bool(reckless_change_lane);

namespace apollo {
namespace planning {

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
 * stop_sign: stop
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
}

}  // namespace planning
}  // namespace apollo

TMAIN;
