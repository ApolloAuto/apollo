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

#ifndef MODULES_CALIBRATION_LIDAR_EX_CHECKER_COMMON_LIDAR_EX_CHECKER_GFLAGS_H_
#define MODULES_CALIBRATION_LIDAR_EX_CHECKER_COMMON_LIDAR_EX_CHECKER_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(node_name);

// the number of cloud count to capture
DECLARE_int32(capture_cloud_count);
// the distance between two clouds
DECLARE_double(capture_distance);

DECLARE_string(adapter_config_filename);

#endif
/* MODULES_CALIBRATION_LIDAR_EX_CHECKER_COMMON_LIDAR_EX_CHECKER_GFLAGS_H_ */
