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

#ifndef MODULES_CANBUS_COMMON_GFLAGS_H_
#define MODULES_CANBUS_COMMON_GFLAGS_H_

#include "gflags/gflags.h"

// System gflags
DECLARE_string(canbus_node_name);
DECLARE_string(hmi_name);

DECLARE_string(adapter_config_filename);

// data file
DECLARE_string(canbus_conf_file);

// Canbus gflags
DECLARE_double(chassis_freq);
DECLARE_int64(min_cmd_interval);

// chassis_detail message publish
DECLARE_bool(enable_chassis_detail_pub);
#endif
