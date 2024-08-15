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

// cmd input check
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms.");
DEFINE_int64(pad_msg_delay_interval, 3,
             "Minimum pad msg command delay interval in s.");
DEFINE_int32(max_control_miss_num, 5, "max control miss num.");
DEFINE_double(control_period, 0.01, "control period in s.");
DEFINE_int32(max_guardian_miss_num, 5, "max guardian miss num.");
DEFINE_double(guardian_period, 0.01, "control period in s.");
DEFINE_bool(use_control_cmd_check, false, "enable control cmd input check.");
DEFINE_bool(use_guardian_cmd_check, false, "nable guardian cmd input check.");
DEFINE_double(estop_brake, 30.0, "brake action when cmd input check error.");

// chassis_detail message publish
DEFINE_bool(enable_chassis_detail_pub, true,
            "Chassis Detail receive message publish");
DEFINE_bool(enable_chassis_detail_sender_pub, true,
            "Chassis Detail sender message publish");

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
DEFINE_int32(chassis_cmd_pending_queue_size, 10,
             "Max control cmd pending queue size");

// enable forward Ultrasonic AEB
DEFINE_bool(enable_aeb, false, "Enable forward Ultrasonic AEB");

// enabel chassis debug mode for such as ignore pad msg timestamp check
DEFINE_bool(chassis_debug_mode, false, "Enable chassis in debug mode");

// vehicle factory dynamic library path and class name
DEFINE_string(load_vehicle_library,
              "/opt/apollo/neo/lib/modules/canbus_vehicle/lincoln/"
              "liblincoln_vehicle_factory_lib.so",
              "Default load vehicle library");
DEFINE_string(load_vehicle_class_name, "LincolnVehicleFactory",
              "Default vehicle factory name");
