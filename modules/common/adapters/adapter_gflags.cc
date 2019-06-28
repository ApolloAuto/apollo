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

#include "modules/common/adapters/adapter_gflags.h"

DEFINE_bool(enable_adapter_dump, false,
            "Whether enable dumping the messages to "
            "/tmp/adapters/<topic_name>/<seq_num>.txt for debugging purposes.");
DEFINE_string(gps_topic, "/apollo/sensor/gnss/odometry", "GPS topic name");
DEFINE_string(imu_topic, "/apollo/sensor/gnss/corrected_imu", "IMU topic name");
DEFINE_string(raw_imu_topic, "/apollo/sensor/gnss/imu", "Raw IMU topic name");
DEFINE_string(chassis_topic, "/apollo/canbus/chassis", "chassis topic name");
DEFINE_string(chassis_detail_topic, "/apollo/canbus/chassis_detail",
              "chassis detail topic name");
DEFINE_string(localization_topic, "/apollo/localization/pose",
              "localization topic name");
DEFINE_string(planning_trajectory_topic, "/apollo/planning",
              "planning trajectory topic name");
DEFINE_string(planning_pad_topic, "/apollo/planning/pad",
              "planning pad topic name");
DEFINE_string(monitor_topic, "/apollo/monitor", "Monitor");
DEFINE_string(pad_topic, "/apollo/control/pad",
              "control pad message topic name");
DEFINE_string(control_command_topic, "/apollo/control",
              "control command topic name");
DEFINE_string(pointcloud_topic,
              "/apollo/sensor/lidar128/compensator/PointCloud2",
              "pointcloud topic name");
DEFINE_string(pointcloud_16_front_up_topic,
              "/apollo/sensor/lidar16/front/up/compensator/PointCloud2",
              "Front up 16 beam Lidar pointcloud topic name");
DEFINE_string(pointcloud_64_topic,
              "/apollo/sensor/velodyne64/compensator/PointCloud2",
              "pointcloud topic name");
DEFINE_string(pointcloud_128_topic,
              "/apollo/sensor/lidar128/compensator/PointCloud2",
              "pointcloud topic name for 128 beam lidar");
DEFINE_string(pointcloud_raw_topic, "/apollo/sensor/velodyne64/PointCloud2",
              "pointcloud raw topic name");
DEFINE_string(velodyne_raw_topic,
              "/apollo/sensor/velodyne64/VelodyneScanUnified",
              "velodyne64 raw data topic name");
DEFINE_string(pointcloud_fusion_topic,
              "/apollo/sensor/velodyne64/fusion/PointCloud2",
              "pointcloud fusion topic name");
DEFINE_string(vlp16_pointcloud_topic,
              "/apollo/sensor/velodyne16/compensator/PointCloud2",
              "16 beam Lidar pointcloud topic name");
DEFINE_string(lidar_16_front_center_topic,
              "/apollo/sensor/lidar16/front/center/PointCloud2",
              "front center 16 beam lidar topic name");
DEFINE_string(lidar_16_front_up_topic,
              "/apollo/sensor/lidar16/front/up/PointCloud2",
              "front up 16 beam lidar topic name");
DEFINE_string(lidar_16_rear_left_topic,
              "/apollo/sensor/lidar16/rear/left/PointCloud2",
              "rear left 16 beam lidar topic name");
DEFINE_string(lidar_16_rear_right_topic,
              "/apollo/sensor/lidar16/rear/right/PointCloud2",
              "rear right 16 beam lidar topic name");
DEFINE_string(lidar_16_fusion_topic,
              "/apollo/sensor/lidar16/fusion/PointCloud2",
              "16 beam lidar fusion topic name");
DEFINE_string(lidar_16_fusion_compensator_topic,
              "/apollo/sensor/lidar16/fusion/compensator/PointCloud2",
              "16 beam lidar fusion compensator topic name");
DEFINE_string(lidar_128_topic, "/apollo/sensor/lidar128/PointCloud2",
              "128 beam lidar topic name");
DEFINE_string(prediction_topic, "/apollo/prediction", "prediction topic name");
DEFINE_string(perception_obstacle_topic, "/apollo/perception/obstacles",
              "perception obstacle topic name");
DEFINE_string(drive_event_topic, "/apollo/drive_event",
              "drive event topic name");
DEFINE_string(traffic_light_detection_topic, "/apollo/perception/traffic_light",
              "traffic light detection topic name");
DEFINE_string(perception_lane_mask_segmentation_topic,
              "/apollo/perception/lane_mask",
              "lane mask segmentation topic name");
DEFINE_string(routing_request_topic, "/apollo/routing_request",
              "routing request topic name");
DEFINE_string(routing_response_topic, "/apollo/routing_response",
              "routing response topic name");
DEFINE_string(routing_response_history_topic,
              "/apollo/routing_response_history",
              "routing response history topic name");
DEFINE_string(relative_odometry_topic, "/apollo/calibration/relative_odometry",
              "relative odometry topic name");
DEFINE_string(ins_stat_topic, "/apollo/sensor/gnss/ins_stat",
              "ins stat topic name");
DEFINE_string(ins_status_topic, "/apollo/sensor/gnss/ins_status",
              "ins status topic name");
DEFINE_string(gnss_status_topic, "/apollo/sensor/gnss/gnss_status",
              "gnss status topic name");
DEFINE_string(system_status_topic, "/apollo/monitor/system_status",
              "System status topic name");
DEFINE_string(static_info_topic, "/apollo/monitor/static_info",
              "Static info topic name");
DEFINE_string(mobileye_topic, "/apollo/sensor/mobileye", "mobileye topic name");
DEFINE_string(delphi_esr_topic, "/apollo/sensor/delphi_esr",
              "delphi esr radar topic name");
DEFINE_string(conti_radar_topic, "/apollo/sensor/conti_radar",
              "continental radar topic name");
DEFINE_string(racobit_radar_topic, "/apollo/sensor/racobit_radar",
              "racobit radar topic name");
DEFINE_string(ultrasonic_radar_topic, "/apollo/sensor/ultrasonic_radar",
              "ultrasonic esr radar topic name");
DEFINE_string(front_radar_topic, "/apollo/sensor/radar/front",
              "front radar topic name");
DEFINE_string(rear_radar_topic, "/apollo/sensor/radar/rear",
              "rear radar topic name");
// TODO(Authors): Change the topic name
DEFINE_string(compressed_image_topic, "camera/image_raw",
              "CompressedImage topic name");
DEFINE_string(image_front_topic, "/apollo/sensor/camera/front_6mm/image",
              "front camera image topic name for obstacles from camera");
DEFINE_string(image_short_topic,
              "/apollo/sensor/camera/front_6mm/image/compressed",
              "short camera image topic name");
DEFINE_string(image_long_topic, "/apollo/sensor/camera/traffic/image_long",
              "long camera image topic name");
DEFINE_string(image_usb_cam_topic, "/apollo/sensor/camera/image_usb_cam",
              "USB camera image topic name");
DEFINE_string(camera_image_long_topic, "/apollo/sensor/camera/image_long",
              "long camera image topic name");
DEFINE_string(camera_image_short_topic, "/apollo/sensor/camera/image_short",
              "short camera image topic name");
DEFINE_string(camera_front_6mm_compressed_topic,
              "/apollo/sensor/camera/front_6mm/image/compressed",
              "front 6mm camera compressed topic name");
DEFINE_string(camera_front_12mm_compressed_topic,
              "/apollo/sensor/camera/front_12mm/image/compressed",
              "front 12mm camera compressed topic name");
DEFINE_string(camera_left_fisheye_compressed_topic,
              "/apollo/sensor/camera/left_fisheye/image/compressed",
              "left fisheye camera compressed topic name");
DEFINE_string(camera_right_fisheye_compressed_topic,
              "/apollo/sensor/camera/right_fisheye/image/compressed",
              "right fisheye camera compressed topic name");
DEFINE_string(camera_rear_6mm_compressed_topic,
              "/apollo/sensor/camera/rear_6mm/image/compressed",
              "front 6mm camera compressed topic name");
DEFINE_string(camera_front_6mm_video_compressed_topic,
              "/apollo/sensor/camera/front_6mm/video/compressed",
              "front 6mm camera video compressed topic name");
DEFINE_string(camera_front_12mm_video_compressed_topic,
              "/apollo/sensor/camera/front_12mm/video/compressed",
              "front 12mm camera video compressed topic name");
DEFINE_string(camera_left_fisheye_video_compressed_topic,
              "/apollo/sensor/camera/left_fisheye/video/compressed",
              "left fisheye camera video compressed topic name");
DEFINE_string(camera_right_fisheye_video_compressed_topic,
              "/apollo/sensor/camera/right_fisheye/video/compressed",
              "right fisheye camera video compressed topic name");
DEFINE_string(camera_rear_6mm_video_compressed_topic,
              "/apollo/sensor/camera/rear_6mm/video/compressed",
              "front 6mm camera video compressed topic name");
DEFINE_string(gnss_rtk_obs_topic, "/apollo/sensor/gnss/rtk_obs",
              "Gnss rtk observation topic name");
DEFINE_string(gnss_rtk_eph_topic, "/apollo/sensor/gnss/rtk_eph",
              "Gnss rtk ephemeris topic name");
DEFINE_string(gnss_best_pose_topic, "/apollo/sensor/gnss/best_pose",
              "Gnss rtk best gnss pose");
DEFINE_string(localization_gnss_topic, "/apollo/localization/msf_gnss",
              "Gnss localization measurement topic name");
DEFINE_string(localization_lidar_topic, "/apollo/localization/msf_lidar",
              "Lidar localization measurement topic name");
DEFINE_string(localization_ndt_topic, "/apollo/localization/ndt_lidar",
              "NDT localization lidar measurement topic name");
DEFINE_string(localization_sins_pva_topic, "/apollo/localization/msf_sins_pva",
              "Localization sins pva topic name");
DEFINE_string(localization_msf_status, "/apollo/localization/msf_status",
              "msf localization status");
DEFINE_string(relative_map_topic, "/apollo/relative_map", "relative map");
DEFINE_string(navigation_topic, "/apollo/navigation", "navigation");
DEFINE_string(hmi_status_topic, "/apollo/hmi/status", "HMI status topic name.");
DEFINE_string(audio_capture_topic, "/apollo/hmi/audio_capture",
              "HMI audio capture topic name.");
DEFINE_string(v2x_obstacle_topic, "/apollo/v2x/obstacles",
              "v2x obstacles topic name");
DEFINE_string(v2x_trafficlight_topic, "/apollo/v2x/traffic_light",
              "v2x trafficlight topic name");
// For pandora.
DEFINE_string(pandora_pointcloud_topic,
              "/apollo/sensor/pandora/hesai40/PointCloud2",
              "pandora pointcloud topic name");
DEFINE_string(pandora_camera_front_color_topic,
              "/apollo/sensor/pandora/camera/front_color",
              "pandora front color camera topic name");
DEFINE_string(pandora_camera_right_gray_topic,
              "/apollo/sensor/pandora/camera/right_gray",
              "pandora right gray camera topic name");
DEFINE_string(pandora_camera_left_gray_topic,
              "/apollo/sensor/pandora/camera/left_gray",
              "pandora left gray camera topic name");
DEFINE_string(pandora_camera_front_gray_topic,
              "/apollo/sensor/pandora/camera/front_gray",
              "pandora front gray camera topic name");
DEFINE_string(pandora_camera_back_gray_topic,
              "/apollo/sensor/pandora/camera/back_gray",
              "pandora back gray camera topic name");
DEFINE_string(guardian_topic, "/apollo/guardian", "Guardian topic.");
DEFINE_string(gnss_raw_data_topic, "/apollo/sensor/gnss/raw_data",
              "gnss raw data topic name");
DEFINE_string(stream_status_topic, "/apollo/sensor/gnss/stream_status",
              "gnss stream status topic name");
DEFINE_string(heading_topic, "/apollo/sensor/gnss/heading",
              "gnss heading topic name");
DEFINE_string(rtcm_data_topic, "/apollo/sensor/gnss/rtcm_data",
              "gnss rtcm data topic name");
DEFINE_string(tf_topic, "/tf", "Transform topic.");
DEFINE_string(tf_static_topic, "/tf_static", "Transform static topic.");
DEFINE_string(recorder_status_topic, "/apollo/data/recorder/status",
              "Recorder status topic.");
