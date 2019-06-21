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

DECLARE_bool(enable_adapter_dump);
DECLARE_string(gps_topic);
DECLARE_string(imu_topic);
DECLARE_string(raw_imu_topic);
DECLARE_string(chassis_topic);
DECLARE_string(chassis_detail_topic);
DECLARE_string(localization_topic);
DECLARE_string(planning_trajectory_topic);
DECLARE_string(planning_pad_topic);
DECLARE_string(monitor_topic);
DECLARE_string(pad_topic);
DECLARE_string(control_command_topic);
DECLARE_string(pointcloud_topic);
DECLARE_string(pointcloud_16_front_up_topic);
DECLARE_string(pointcloud_64_topic);
DECLARE_string(pointcloud_128_topic);
DECLARE_string(pointcloud_raw_topic);
DECLARE_string(velodyne_raw_topic);
DECLARE_string(pointcloud_fusion_topic);
DECLARE_string(vlp16_pointcloud_topic);
DECLARE_string(lidar_16_front_center_topic);
DECLARE_string(lidar_16_front_up_topic);
DECLARE_string(lidar_16_rear_left_topic);
DECLARE_string(lidar_16_rear_right_topic);
DECLARE_string(lidar_16_fusion_topic);
DECLARE_string(lidar_16_fusion_compensator_topic);
DECLARE_string(lidar_128_topic);
DECLARE_string(prediction_topic);
DECLARE_string(perception_obstacle_topic);
DECLARE_string(drive_event_topic);
DECLARE_string(traffic_light_detection_topic);
DECLARE_string(perception_lane_mask_segmentation_topic);
DECLARE_string(routing_request_topic);
DECLARE_string(routing_response_topic);
DECLARE_string(routing_response_history_topic);
DECLARE_string(relative_odometry_topic);
DECLARE_string(ins_stat_topic);
DECLARE_string(ins_status_topic);
DECLARE_string(gnss_status_topic);
DECLARE_string(system_status_topic);
DECLARE_string(static_info_topic);
DECLARE_string(mobileye_topic);
DECLARE_string(delphi_esr_topic);
DECLARE_string(conti_radar_topic);
DECLARE_string(racobit_radar_topic);
DECLARE_string(ultrasonic_radar_topic);
DECLARE_string(front_radar_topic);
DECLARE_string(rear_radar_topic);
DECLARE_string(compressed_image_topic);
DECLARE_string(image_front_topic);
DECLARE_string(image_short_topic);
DECLARE_string(image_long_topic);
DECLARE_string(camera_image_long_topic);
DECLARE_string(camera_image_short_topic);
DECLARE_string(camera_front_6mm_compressed_topic);
DECLARE_string(camera_front_12mm_compressed_topic);
DECLARE_string(camera_left_fisheye_compressed_topic);
DECLARE_string(camera_right_fisheye_compressed_topic);
DECLARE_string(camera_rear_6mm_compressed_topic);
DECLARE_string(camera_front_6mm_video_compressed_topic);
DECLARE_string(camera_front_12mm_video_compressed_topic);
DECLARE_string(camera_left_fisheye_video_compressed_topic);
DECLARE_string(camera_right_fisheye_video_compressed_topic);
DECLARE_string(camera_rear_6mm_video_compressed_topic);
DECLARE_string(gnss_rtk_obs_topic);
DECLARE_string(gnss_rtk_eph_topic);
DECLARE_string(gnss_best_pose_topic);
DECLARE_string(localization_gnss_topic);
DECLARE_string(localization_lidar_topic);
DECLARE_string(localization_ndt_topic);
DECLARE_string(localization_sins_pva_topic);
DECLARE_string(localization_msf_status);
DECLARE_string(relative_map_topic);
DECLARE_string(navigation_topic);
DECLARE_string(hmi_status_topic);
DECLARE_string(audio_capture_topic);
DECLARE_string(v2x_obstacle_topic);
DECLARE_string(v2x_trafficlight_topic);
// For pandora.
DECLARE_string(pandora_pointcloud_topic);
DECLARE_string(pandora_camera_front_color_topic);
DECLARE_string(pandora_camera_right_gray_topic);
DECLARE_string(pandora_camera_left_gray_topic);
DECLARE_string(pandora_camera_front_gray_topic);
DECLARE_string(pandora_camera_back_gray_topic);
DECLARE_string(gnss_raw_data_topic);
DECLARE_string(stream_status_topic);
DECLARE_string(heading_topic);
DECLARE_string(rtcm_data_topic);

// Guardian topic
DECLARE_string(guardian_topic);

// Transform topic
DECLARE_string(tf_topic);
DECLARE_string(tf_static_topic);

// Recorder status topic
DECLARE_string(recorder_status_topic);
