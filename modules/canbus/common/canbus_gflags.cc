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
DEFINE_string(node_name, "chassis", "The chassis module name in proto");
DEFINE_string(hmi_name, "canbus", "Module name in HMI");

DEFINE_string(adapter_config_filename, "modules/canbus/conf/adapter.conf",
              "The adapter config file");

// data file
DEFINE_string(canbus_conf_file, "modules/canbus/conf/canbus_conf.pb.txt",
              "Default canbus conf file");

// Canbus gflags
DEFINE_double(chassis_freq, 100, "Chassis feedback timer frequency.");
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in us.");

// chassis_detail message publish
DEFINE_bool(enable_chassis_detail_pub, false, "Chassis Detail message publish");
