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

namespace apollo {
namespace planning {

/**
 * @class SunnyvaleLoopTest
 * @brief This is an integration test that uses the sunnyvale_loop map.
 */

class NavigationModeTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    FLAGS_use_navigation_mode = true;
    FLAGS_test_data_dir = "modules/planning/testdata/navigation_mode_test";
  }
};

/*
 * test stop for not-nudgable obstacle
 * A cruise test case
 */
TEST_F(NavigationModeTest, cruise) {
  std::string seq_num = "1";
  FLAGS_enable_prediction = false;
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  FLAGS_test_relative_map_file = seq_num + "_relative_map.pb.txt";
  PlanningTestBase::SetUp();
  RUN_GOLDEN_TEST(0);
}
}  // namespace planning
}  // namespace apollo

TMAIN;
