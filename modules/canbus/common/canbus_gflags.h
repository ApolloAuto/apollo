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

#pragma once

#include "gflags/gflags.h"

// System gflags
DECLARE_string(canbus_node_name);
DECLARE_string(canbus_module_name);

// data file
DECLARE_string(canbus_conf_file);

// Canbus gflags
DECLARE_double(chassis_freq);

// cmd input check
DECLARE_int64(min_cmd_interval);
DECLARE_int64(pad_msg_delay_interval);
DECLARE_int32(max_control_miss_num);
DECLARE_double(control_period);
DECLARE_int32(max_guardian_miss_num);
DECLARE_double(guardian_period);
DECLARE_bool(use_control_cmd_check);
DECLARE_bool(use_guardian_cmd_check);
DECLARE_double(estop_brake);

// chassis_detail message publish
DECLARE_bool(enable_chassis_detail_pub);
DECLARE_bool(enable_chassis_detail_sender_pub);

// canbus test files
DECLARE_string(canbus_test_file);

// canbus test files
DECLARE_bool(receive_guardian);

DECLARE_int32(guardian_cmd_pending_queue_size);
DECLARE_int32(control_cmd_pending_queue_size);
DECLARE_int32(chassis_cmd_pending_queue_size);

// enable forward Ultrasonic AEB
DECLARE_bool(enable_aeb);

// enabel chassis debug mode for such as ignore pad msg timestamp check
DECLARE_bool(chassis_debug_mode);

// vehicle factory dynamic library path and class name
DECLARE_string(load_vehicle_library);
DECLARE_string(load_vehicle_class_name);
