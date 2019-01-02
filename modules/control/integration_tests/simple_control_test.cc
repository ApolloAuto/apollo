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

#include "cyber/common/log.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/integration_tests/control_test_base.h"

namespace apollo {
namespace control {

class SimpleControlTest : public ControlTestBase {
 public:
  virtual void SetUp() {
    FLAGS_test_data_dir =
        "/apollo/modules/control/testdata/simple_control_test/";
  }
};

TEST_F(SimpleControlTest, simple_test) {
  FLAGS_enable_csv_debug = true;
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  ControlTestBase::SetUp();
  RUN_GOLDEN_TEST;
}

TEST_F(SimpleControlTest, state_exact_match) {
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  ControlTestBase::SetUp();
  RUN_GOLDEN_TEST;
}

TEST_F(SimpleControlTest, pad_reset) {
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "2_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  ControlTestBase::SetUp();
  RUN_GOLDEN_TEST;
}

TEST_F(SimpleControlTest, monitor_fatal) {
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  FLAGS_test_monitor_file = "1_monitor.pb.txt";
  ControlTestBase::SetUp();
  RUN_GOLDEN_TEST;
}

}  // namespace control
}  // namespace apollo
