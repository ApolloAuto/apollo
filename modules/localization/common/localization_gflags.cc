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
              "/apollo/modules/localization/conf/rtk_adapter.conf",
              "rtk adapter configuration");

DEFINE_string(localization_config_file,
              "/apollo/modules/localization/conf/localization_config.pb.txt",
              "localization config file");

DEFINE_string(msf_adapter_config_file,
              "/apollo/modules/localization/conf/msf_adapter.conf",
              "msf adapter configuration");

DEFINE_string(msf_visual_adapter_config_file,
              "/apollo/modules/localization/conf/msf_visual_adapter.conf",
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
DEFINE_string(lidar_extrinsics_file,
              "/apollo/modules/localization/msf/params/velodyne_params/"
              "velodyne64_novatel_extrinsics_example.yaml",
              "Lidar extrinsics parameter file.");
DEFINE_string(lidar_height_file,
              "/apollo/modules/localization/msf/params/velodyne_params/"
              "velodyne64_height.yaml",
              "Velodyne extrinsic path for the vehicle in use, "
              "where <ros> is the placeholder of ROS root.");
DEFINE_double(lidar_height_default, 1.80,
              "The height from the center of velodyne to ground.");
DEFINE_int32(
    lidar_localization_mode, 2,
    "Localization mode, 0 for intensity, 1 for altitude, 2 for fusion.");
DEFINE_int32(lidar_yaw_align_mode, 2,
             "image yaw align mode, 0 for align off, "
             "1 for fusion, 2 for fusion with multithread.");
DEFINE_int32(lidar_filter_size, 17, "Lidar filter size");
DEFINE_double(lidar_imu_max_delay_time, 0.4,
              "Lidar msg and imu msg max delay time");
DEFINE_double(lidar_map_coverage_theshold, 0.9,
              "Threshold to detect whether vehicle is out of map");
DEFINE_bool(lidar_debug_log_flag, false, "Lidar Debug switch.");
DEFINE_int32(point_cloud_step, 2, "Point cloud step");
DEFINE_bool(if_use_avx, false,
            "if use avx to accelerate lidar localization, "
            "need cpu to support AVX intel instrinsics");

// integ module
DEFINE_bool(integ_ins_can_self_align, false, "");
DEFINE_bool(integ_sins_align_with_vel, true, "");
DEFINE_bool(integ_sins_state_check, false, "");
DEFINE_double(integ_sins_state_span_time, 60.0, "");
DEFINE_double(integ_sins_state_pos_std, 1.0, "");
DEFINE_double(vel_threshold_get_yaw, 5.0, "");

// gnss module
DEFINE_bool(enable_ins_aid_rtk, false, "");
DEFINE_string(eph_buffer_path, "", "");
DEFINE_string(
    ant_imu_leverarm_file,
    "/apollo/modules/localization/msf/params/gnss_params/ant_imu_leverarm.yaml",
    "Ant to imu leferarm.");
DEFINE_bool(gnss_debug_log_flag, false, "Gnss Debug switch.");
DEFINE_bool(if_imuant_from_file, true, "Use imu ant from gnss configure file.");
DEFINE_double(imu_to_ant_offset_x, 0.0, "Imu ant offset x");
DEFINE_double(imu_to_ant_offset_y, 0.0, "Imu ant offset y");
DEFINE_double(imu_to_ant_offset_z, 0.0, "Imu ant offset z");
DEFINE_double(imu_to_ant_offset_ux, 0.0, "Imu ant offset x uncertainty");
DEFINE_double(imu_to_ant_offset_uy, 0.0, "Imu ant offset y uncertainty");
DEFINE_double(imu_to_ant_offset_uz, 0.0, "Imu ant offset z uncertainty");

// common
DEFINE_double(imu_rate, 1.0, "");
DEFINE_bool(if_utm_zone_id_from_folder, true,
            "load utm zone id from local map folder");
DEFINE_bool(trans_gpstime_to_utctime, true, "");
DEFINE_int32(gnss_mode, 0, "GNSS Mode, 0 for bestgnss pose, 1 for self gnss.");
DEFINE_bool(imu_coord_rfu, true, "Right/forward/up");
DEFINE_bool(gnss_only_init, false,
            "Whether use bestgnsspose as measure after initializaiton.");
DEFINE_bool(enable_lidar_localization, true,
            "Enable lidar-based localization.");

DEFINE_string(lidar_topic, "/apollo/sensor/lidar128/compensator/PointCloud2",
              "lidar pointcloud topic");
DEFINE_string(broadcast_tf_frame_id, "world", "world frame id in tf");
DEFINE_string(broadcast_tf_child_frame_id, "localization",
              "localization frame id in tf");
// imu vehicle extrinsic
DEFINE_string(vehicle_imu_file,
              "/apollo/modules/localization/msf/params"
              "/vehicle_params/vehicle_imu_extrinsics.yaml",
              "Vehicle coord to imu coord.");
DEFINE_bool(if_vehicle_imu_from_file, true,
            "Whether load vehicle imu extrinsic from yaml file");
DEFINE_double(imu_vehicle_qx, 0.0, "Vehicle imu quaternion qx");
DEFINE_double(imu_vehicle_qy, 0.0, "Vehicle imu quaternion qy");
DEFINE_double(imu_vehicle_qz, 0.0, "Vehicle imu quaternion qz");
DEFINE_double(imu_vehicle_qw, 1.0, "Vehicle imu quaternion qw");

// visualization
DEFINE_string(map_visual_dir, "data/map_visual",
              "The path of map_visual folder.");

// Status
DEFINE_double(imu_delay_time_threshold_1, 0.1,
              "imu delay time is larger than 0.1s");
DEFINE_double(imu_delay_time_threshold_2, 0.05,
              "imu delay time is larger than 0.05s");
DEFINE_double(imu_delay_time_threshold_3, 0.02,
              "imu delay time is larger than 0.02s");

DEFINE_double(imu_missing_time_threshold_1, 0.1,
              "imu missing time is larger than 0.1s");
DEFINE_double(imu_missing_time_threshold_2, 0.05,
              "imu missing time is larger than 0.05s");
DEFINE_double(imu_missing_time_threshold_3, 0.01,
              "imu missing time is larger than 0.01s");

DEFINE_double(bestgnsspose_loss_time_threshold, 2.0,
              "threshold for time gap between imu and bestgnsspose time");
DEFINE_double(lidar_loss_time_threshold, 2.0,
              "threshold for time gap between imu and lidar pose time");

DEFINE_double(localization_std_x_threshold_1, 0.15,
              "threshold for lateral std of localization result");
DEFINE_double(localization_std_y_threshold_1, 0.15,
              "threshold for longitudinal std of localization result");

DEFINE_double(localization_std_x_threshold_2, 0.3,
              "threshold for lateral std of localization result");
DEFINE_double(localization_std_y_threshold_2, 0.3,
              "threshold for longitudinal std of localization result");

// ndt localization
DEFINE_string(ndt_map_dir, "ndt_map", "subdirectory for ndt map");
DEFINE_bool(ndt_debug_log_flag, false, "NDT Localization log switch");
DEFINE_double(online_resolution, 2.0, "NDT online pointcloud resolution");
DEFINE_int32(ndt_max_iterations, 10, "maximum iterations for NDT matching");
DEFINE_double(ndt_target_resolution, 1.0,
              "target resolution for ndt localization");
DEFINE_double(ndt_line_search_step_size, 0.1,
              "line search step size for ndt matching");
DEFINE_double(ndt_transformation_epsilon, 0.01,
              "iteration convergence condition on transformation");
DEFINE_int32(ndt_filter_size_x, 48, "x size for ndt searching area");
DEFINE_int32(ndt_filter_size_y, 48, "y size for ndt searching area");
DEFINE_int32(ndt_bad_score_count_threshold, 10,
             "count for continuous bad ndt fitness score");
DEFINE_double(ndt_warnning_ndt_score, 1.0,
              "warnning ndt fitness score threshold");
DEFINE_double(ndt_error_ndt_score, 2.0, "error ndt fitness score threshold");
