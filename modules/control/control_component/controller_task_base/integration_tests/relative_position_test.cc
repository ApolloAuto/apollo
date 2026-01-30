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

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/util/util.h"
#include "modules/control/control_component/common/control_gflags.h"
#include "modules/control/control_component/controller_task_base/integration_tests/control_test_base.h"

namespace apollo {
namespace control {

class RelativePositionTest : public ControlTestBase {
 public:
  virtual void SetUp() {
    FLAGS_test_data_dir =
        "/apollo/modules/control/control_component/testdata/"
        "relative_position_test/";
    FLAGS_use_navigation_mode = true;
    FLAGS_is_control_ut_test_mode = true;
  }
};

TEST_F(RelativePositionTest, simple_left) {
  FLAGS_enable_csv_debug = true;
  FLAGS_test_localization_file = "0_apollo_localization_pose.pb.txt";
  FLAGS_test_pad_file = "0_pad.pb.txt";
  FLAGS_test_planning_file = "0_apollo_planning.pb.txt";
  FLAGS_test_chassis_file = "0_apollo_canbus_chassis.pb.txt";
  ControlTestBase::SetUp();
  ControlTestBase::SetUpTestCase();
  RUN_GOLDEN_TEST;
}

}  // namespace control
}  // namespace apollo
