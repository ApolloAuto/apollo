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

DEFINE_string(msf_visual_adapter_config_file,
              "modules/localization/conf/msf_visual_adapter.conf",
              "msf visualization adapter configuration");

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

// msf parame
DEFINE_string(local_map_name, "local_map", "The path of localization map.");
DEFINE_string(lidar_height_file,
              "<ros>/share/velodyne_pointcloud/params/"
              "velodyne64_height_example.yaml",
              "Velodyne extrinsic path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");
DEFINE_int32(
    lidar_localization_mode, 2,
    "Localization mode, 0 for intensity, 1 for altitude, 2 for fusion.");
DEFINE_int32(lidar_filter_size, 11, "Lidar filter size");
DEFINE_int32(lidar_thread_num, 2, "Lidar thread number");
DEFINE_double(lidar_imu_max_delay_time, 0.4,
              "Lidar msg and imu msg max delay time");
DEFINE_double(lidar_map_coverage_theshold, 0.9,
              "Threshold to detect wether vehicle is out of map");
DEFINE_bool(lidar_debug_log_flag, false, "Lidar Debug switch.");
DEFINE_int32(point_cloud_step, 2, "Point cloud step");

// integ module
DEFINE_bool(integ_ins_can_self_align, false, "");
DEFINE_bool(integ_sins_align_with_vel, true, "");
DEFINE_double(vel_threshold_get_yaw, 5.0, "");
DEFINE_bool(integ_debug_log_flag, false, "");
DEFINE_string(broadcast_tf2_frame_id, "world",
              "The frame id used to broadcast the localization result.");
DEFINE_string(broadcast_tf2_child_frame_id, "localization_100hz",
              "The child frame id used to broadcast the localization result.");

// gnss module
DEFINE_bool(enable_ins_aid_rtk, false, "");
DEFINE_bool(enable_auto_save_eph_file, true, "");
DEFINE_string(eph_buffer_path, "", "");
DEFINE_bool(gnss_debug_log_flag, false, "Gnss Debug switch.");
DEFINE_bool(imuant_from_gnss_conf_file, true,
            "Use imu ant from gnss configure file.");
DEFINE_double(imu_to_ant_offset_x, 0.0, "Imu ant offset x");
DEFINE_double(imu_to_ant_offset_y, 0.0, "Imu ant offset y");
DEFINE_double(imu_to_ant_offset_z, 0.0, "Imu ant offset z");
DEFINE_double(imu_to_ant_offset_ux, 0.0, "Imu ant offset x uncertainty");
DEFINE_double(imu_to_ant_offset_uy, 0.0, "Imu ant offset y uncertainty");
DEFINE_double(imu_to_ant_offset_uz, 0.0, "Imu ant offset z uncertainty");

// common
DEFINE_double(imu_rate, 1.0, "");
DEFINE_int32(local_utm_zone_id, 50, "UTM zone id");
DEFINE_bool(trans_gpstime_to_utctime, true, "");
DEFINE_int32(gnss_mode, 0, "GNSS Mode, 0 for bestgnss pose, 1 for self gnss.");
DEFINE_bool(imu_coord_rfu, true, "Right/forward/up");
DEFINE_bool(gnss_only_init, false,
            "Whether use bestgnsspose as measure after initializaiton.");

// debug
DEFINE_bool(use_visualize, false, "");
