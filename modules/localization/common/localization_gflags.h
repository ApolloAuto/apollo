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

#ifndef MODULES_LOCALIZATION_COMMON_LOCALIZATION_GFLAGS_H_
#define MODULES_LOCALIZATION_COMMON_LOCALIZATION_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(localization_module_name);

DECLARE_double(localization_publish_freq);

DECLARE_string(localization_config_file);
DECLARE_string(rtk_adapter_config_file);
DECLARE_string(msf_adapter_config_file);

DECLARE_bool(enable_gps_imu_interprolate);
DECLARE_bool(enable_map_reference_unify);
DECLARE_bool(enable_watchdog);

DECLARE_double(gps_time_delay_tolerance);
DECLARE_double(gps_imu_timestamp_sec_diff_tolerance);
DECLARE_double(timestamp_sec_tolerance);

DECLARE_double(map_offset_x);
DECLARE_double(map_offset_y);
DECLARE_double(map_offset_z);

DECLARE_int32(report_threshold_err_num);
DECLARE_double(report_gps_imu_time_diff_threshold);

DECLARE_bool(enable_gps_timestamp);

// lidar localization
DECLARE_string(map_path);
DECLARE_string(lidar_extrinsic_file);
DECLARE_string(lidar_height_file);
DECLARE_bool(debug_log_flag);
DECLARE_bool(is_locator_available);
DECLARE_string(broadcast_tf2_frame_id_lidar);
DECLARE_string(broadcast_tf_child_frame_id_lidar);
DECLARE_string(query_tf2_target_frame_id_lidar);
DECLARE_string(query_tf2_source_frame_id_lidar);
DECLARE_string(publish_frame_id_lidar);
DECLARE_int32(localization_mode);
DECLARE_int32(tf2_buffer_expire_time);
DECLARE_int32(local_utm_zone_id);
DECLARE_double(map_coverage_theshold);

#endif  // MODULES_LOCALIZATION_COMMON_LOCALIZATION_GFLAGS_H_
