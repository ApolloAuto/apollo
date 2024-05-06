/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/data/tools/smart_recorder/channel_pool.h"

#include <algorithm>

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace data {

ChannelPool::ChannelPool() {
  small_channels_ = {FLAGS_chassis_topic,
                     FLAGS_chassis_detail_topic,
                     FLAGS_control_command_topic,
                     FLAGS_pad_topic,
                     FLAGS_drive_event_topic,
                     FLAGS_guardian_topic,
                     FLAGS_hmi_status_topic,
                     FLAGS_localization_topic,
                     FLAGS_localization_gnss_topic,
                     FLAGS_localization_lidar_topic,
                     FLAGS_localization_msf_status,
                     FLAGS_monitor_topic,
                     FLAGS_system_status_topic,
                     FLAGS_navigation_topic,
                     FLAGS_perception_obstacle_topic,
                     FLAGS_traffic_light_detection_topic,
                     FLAGS_planning_trajectory_topic,
                     FLAGS_prediction_topic,
                     FLAGS_relative_map_topic,
                     FLAGS_routing_response_history_topic,
                     FLAGS_gnss_best_pose_topic,
                     FLAGS_imu_topic,
                     FLAGS_gnss_status_topic,
                     FLAGS_raw_imu_topic,
                     FLAGS_ins_stat_topic,
                     FLAGS_gps_topic,
                     FLAGS_gnss_raw_data_topic,
                     FLAGS_gnss_rtk_eph_topic,
                     FLAGS_gnss_rtk_obs_topic,
                     FLAGS_heading_topic,
                     FLAGS_tf_topic,
                     FLAGS_tf_static_topic,
                     FLAGS_recorder_status_topic,
                     FLAGS_latency_recording_topic,
                     FLAGS_latency_reporting_topic};
  large_channels_ = {FLAGS_camera_front_12mm_compressed_topic,
                     FLAGS_camera_front_6mm_compressed_topic,
                     FLAGS_camera_left_fisheye_compressed_topic,
                     FLAGS_camera_right_fisheye_compressed_topic,
                     FLAGS_camera_rear_6mm_compressed_topic,
                     FLAGS_camera_front_12mm_video_compressed_topic,
                     FLAGS_camera_front_6mm_video_compressed_topic,
                     FLAGS_camera_left_fisheye_video_compressed_topic,
                     FLAGS_camera_right_fisheye_video_compressed_topic,
                     FLAGS_camera_rear_6mm_video_compressed_topic,
                     FLAGS_front_radar_topic,
                     FLAGS_rear_radar_topic,
                     FLAGS_pointcloud_16_topic,
                     FLAGS_pointcloud_16_raw_topic,
                     FLAGS_pointcloud_16_front_left_raw_topic,
                     FLAGS_pointcloud_16_front_right_raw_topic,
                     FLAGS_lidar_16_front_center_topic,
                     FLAGS_lidar_16_front_up_topic,
                     FLAGS_lidar_16_rear_left_topic,
                     FLAGS_lidar_16_rear_right_topic,
                     FLAGS_lidar_16_fusion_topic,
                     FLAGS_lidar_16_fusion_compensator_topic,
                     FLAGS_pointcloud_64_topic,
                     FLAGS_lidar_128_topic,
                     FLAGS_pointcloud_128_topic,
                     FLAGS_mobileye_topic,
                     FLAGS_conti_radar_topic,
                     FLAGS_delphi_esr_topic};
  std::set_union(std::begin(small_channels_), std::end(small_channels_),
                 std::begin(large_channels_), std::end(large_channels_),
                 std::inserter(all_channels_, std::begin(all_channels_)));
}

}  // namespace data
}  // namespace apollo
