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

#include "modules/canbus/common/canbus_gflags.h"

// System gflags
DEFINE_string(canbus_node_name, "chassis", "The chassis module name in proto");
DEFINE_string(canbus_module_name, "canbus_component", "Module name");

// data file
DEFINE_string(canbus_conf_file,
              "/apollo/modules/canbus/conf/canbus_conf.pb.txt",
              "Default canbus conf file");

// Canbus gflags
DEFINE_double(chassis_freq, 100, "Chassis feedback timer frequency.");
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms.");

// chassis_detail message publish
DEFINE_bool(enable_chassis_detail_pub, false, "Chassis Detail message publish");

// canbus test files
DEFINE_string(canbus_test_file,
              "/apollo/modules/canbus/testdata/canbus_test.pb.txt",
              "canbus tester input test file, in ControlCommand pb format.");

// enable receiving guardian command
// TODO(QiL) : depreciate this after test
DEFINE_bool(receive_guardian, false,
            "Enable receiving guardian message on canbus side");

DEFINE_int32(guardian_cmd_pending_queue_size, 10,
             "Max guardian cmd pending queue size");
DEFINE_int32(control_cmd_pending_queue_size, 10,
             "Max control cmd pending queue size");
