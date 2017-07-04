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

/**
 * @file localization_gflags.h
 * @brief The gflags used by localization module
 */

#ifndef MODEULES_LOCALIZATION_COMMON_LOCALIZATION_GFLAGS_H_
#define MODEULES_LOCALIZATION_COMMON_LOCALIZATION_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(localization_module_name);

DECLARE_double(localization_publish_freq);

DECLARE_string(localization_config_file);
DECLARE_string(rtk_adapter_config_file);
DECLARE_string(camera_adapter_config_file);
DECLARE_string(camera_parameter_config_file);

DECLARE_bool(enable_gps_imu_interprolate);
DECLARE_bool(enable_map_reference_unify);
DECLARE_bool(enable_watchdog);

DECLARE_double(gps_time_delay_tolerance);
DECLARE_double(imu_time_delay_tolerance);
DECLARE_double(camera_time_delay_tolerance);
DECLARE_double(gps_imu_timestamp_sec_diff_tolerance);
DECLARE_double(timestamp_sec_tolerance);

DECLARE_double(map_offset_x);
DECLARE_double(map_offset_y);
DECLARE_double(map_offset_z);

DECLARE_int32(monitor_level);

DECLARE_int32(report_threshold_err_num);
DECLARE_double(report_gps_imu_time_diff_threshold);

DECLARE_bool(enable_gps_timestamp);
DECLARE_bool(enable_camera_timestamp);

#endif  // MODEULES_LOCALIZATION_COMMON_LOCALIZATION_GFLAGS_H_
