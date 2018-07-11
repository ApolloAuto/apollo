/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 * Copyright 2018 Baidu X-Lab. Yunhan Jia <jiayunhan@baidu.com>

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

#include <memory>
#include <string>
#include <utility>
#include <iostream>

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "modules/common/log.h"

#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/integration_tests/control_fuzz_base.h"

#include "libfuzzer_macro.h"
#include "modules/planning/proto/planning.pb.h"


using apollo::common::time::Clock;

protobuf_mutator::protobuf::LogSilencer log_silincer;

namespace apollo {
namespace control {

class SimpleControlFuzz : public ControlFuzzBase {
 public:
  virtual void SetUp() {
    FLAGS_test_data_dir = "modules/control/testdata/simple_control_fuzz/";
  }
  void target();
}simple_control_fuzzer;

void SimpleControlFuzz::target() {
  FLAGS_enable_csv_debug = true;
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  ControlFuzzBase::SetUp();
  test_control();
}

/*TEST_F(SimpleControlTest, state_exact_match) {
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
}*/

}  // namespace control
}  // namespace apollo

/******************************************************
* To fuzz certain functions that takes hard-coded file 
* as input, and you don't want to make excessive 
* modification. You can save the mutated message into file
* before calling the target function.  
******************************************************/
DEFINE_PROTO_FUZZER(const apollo::planning::ADCTrajectory& message){
  apollo::control::simple_control_fuzzer.SetUp();
  apollo::control::simple_control_fuzzer.SetUpTestCase();
  apollo::common::util::SetProtoToASCIIFile(message,"modules/control/testdata/simple_control_fuzz/1_planning.pb.txt");
  apollo::control::simple_control_fuzzer.target();
}