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

#include "modules/localization/common/localization_gflags.h"

DEFINE_string(localization_module_name, "localization",
              "localization module name");

DEFINE_double(localization_publish_freq, 100,
              "localization publishing frequency.");

DEFINE_string(rtk_adapter_config_file,
              "modules/localization/conf/rtk_adapter.conf",
              "rtk adapter configuration");

DEFINE_string(localization_config_file,
              "modules/localization/conf/localization_config.pb.txt",
              "localization config file");

DEFINE_string(msf_adapter_config_file,
              "modules/localization/conf/msf_adapter.conf",
              "msf adapter configuration");

// features
DEFINE_bool(enable_gps_imu_interprolate, true, "enable GPU/IMU interprolate");
DEFINE_bool(enable_map_reference_unify, true,
            "enable IMU data convert to map reference");
DEFINE_bool(enable_watchdog, true, "enable watchdog");

DEFINE_double(gps_time_delay_tolerance, 1.0,
              "gps message time delay tolerance (sec)");

DEFINE_double(gps_imu_timestamp_sec_diff_tolerance, 20e-3,
              "gps/imu timestamp diff tolerance (sec)");

DEFINE_double(timestamp_sec_tolerance, 10e-7, "timestamp second tolerance");
// map offset
DEFINE_double(map_offset_x, 0.0, "map_offsite: x");
DEFINE_double(map_offset_y, 0.0, "map_offsite: y");
DEFINE_double(map_offset_z, 0.0, "map_offsite: z");

DEFINE_int32(report_threshold_err_num, 10, "report threshold error num");
DEFINE_double(report_gps_imu_time_diff_threshold, 0.02,
              "report threshold of timestamp diff between gps and imu(sec)");

DEFINE_bool(enable_gps_timestamp, false,
            "True to set gps timestamp as localization header timestamp");

// lidar localization
DEFINE_string(map_path, "../mapdata/local_map",
    "The path of localization map.");
DEFINE_string(lidar_extrinsic_file,
    "/home/caros/ros/share/params/velodyne64_novatel_extrinsics_example.yaml",
    "The path of extrinsics parameter of velodyne64 and imu.");
DEFINE_string(lidar_height_file,
    "/home/caros/ros/share/params/velodyne64_height.yaml",
    "The path of height parameter of lidar.");
DEFINE_bool(debug_log_flag, false, "Debug switch.");
DEFINE_bool(is_locator_available, true, "The status of lidar locator.");
DEFINE_string(broadcast_tf2_frame_id_lidar, "localization_lidar",
    "The child frame id used to broadcast the lidar localization result.");
DEFINE_string(broadcast_tf_child_frame_id_lidar, "world",
    "The child frame id used to broadcast the lidar localization result.");
DEFINE_string(query_tf2_target_frame_id_lidar, "world",
    "The target frame id used to query.");
DEFINE_string(query_tf2_source_frame_id_lidar, "novatel",
    "The source frame id used to query.");
DEFINE_string(publish_frame_id_lidar, "localization_lidar",
    "The frame id used to publish localization result.");
DEFINE_int32(localization_mode, 2,
    "Localization mode, 0 for intensity, 1 for altitude, 2 for fusion.");
DEFINE_int32(tf2_buffer_expire_time, 10,
    "Query Ros TF timeout in ms. ros::Duration time.");
DEFINE_int32(local_utm_zone_id, 50, "UTM zone id.");
DEFINE_double(map_coverage_theshold, 0.9,
    "The valid coverage of pointcloud and map.");
