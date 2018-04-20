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

#include "modules/perception/common/perception_gflags.h"

DEFINE_string(perception_adapter_config_filename,
              "modules/perception/conf/adapter.conf",
              "The adapter config filename");

/// lib/config_manager/config_manager.cc
DEFINE_string(config_manager_path, "./conf/config_manager.config",
              "The ModelConfig config paths file.");
DEFINE_string(work_root, "/apollo/modules/perception/",
              "perception work root direcotry.");

/// obstacle/base/object.cc
DEFINE_bool(is_serialize_point_cloud, false,
            "serialize and output object cloud");

/// obstacle/onboard/hdmap_input.cc
DEFINE_double(map_radius, 60.0, "get map radius of car center");
DEFINE_int32(map_sample_step, 1, "step for sample road boundary points");

/// obstacle/onboard/lidar_process.cc
DEFINE_bool(enable_hdmap_input, false, "enable hdmap input for roi filter");
DEFINE_string(onboard_roi_filter, "DummyROIFilter", "onboard roi filter");
DEFINE_string(onboard_segmentor, "DummySegmentation", "onboard segmentation");
DEFINE_string(onboard_object_builder, "DummyObjectBuilder",
              "onboard object builder");
DEFINE_string(onboard_tracker, "DummyTracker", "onboard tracker");
DEFINE_string(onboard_type_fuser, "DummyTypeFuser", "onboard type fuser");

DEFINE_int32(tf2_buff_in_ms, 10, "the tf2 buff size in ms");
DEFINE_int32(localization_buffer_size, 40, "localization buffer size");
DEFINE_string(lidar_tf2_frame_id, "novatel", "the tf2 transform frame id");
DEFINE_string(lidar_tf2_child_frame_id, "velodyne64",
              "the tf2 transform child frame id");
DEFINE_string(obstacle_module_name, "perception_obstacle",
              "perception obstacle module name");
DEFINE_bool(enable_visualization, false, "enable visualization for debug");

/// obstacle/perception.cc
/* dag streaming config for Apollo 2.0 */
DEFINE_string(dag_config_path, "./conf/dag_streaming.config",
              "Onboard DAG Streaming config.");

/// obstacle/onboard/radar_process_subnode.cc
DEFINE_string(onboard_radar_detector, "DummyRadarDetector",
              "onboard radar detector");
DEFINE_string(radar_tf2_frame_id, "novatel", "the tf2 transform frame id");
DEFINE_string(radar_tf2_child_frame_id, "radar",
              "the tf2 transform child frame id");
DEFINE_double(front_radar_forward_distance, 120.0,
              "get front radar forward distancer");
DEFINE_string(radar_extrinsic_file,
              "modules/perception/data/params/radar_extrinsics.yaml",
              "radar extrinsic file");
DEFINE_string(short_camera_extrinsic_file,
              "modules/perception/data/params/short_camera_extrinsics.yaml",
              "short_camera extrinsic file");

/// obstacle/onboard/camera_process_subnode.cc
// Ex: /apollo/modules/perception/data/yolo_camera_detector_test/test.jpg
DEFINE_string(image_file_path, "", "Debug image file");
DEFINE_bool(image_file_debug, false, "Debug ROS to CV image");

/// modules/perception/lib/config_manager/calibration_config_manager.cc
DEFINE_string(front_camera_extrinsics_file,
              "modules/perception/data/params/front_camera_extrinsics.yaml",
              "front_camera extrinsic file");
DEFINE_string(front_camera_intrinsics_file,
              "modules/perception/data/params/front_camera_intrinsics.yaml",
              "front_camera intrinsic file");

/// obstacle/onboard/fusion_subnode.cc
DEFINE_string(onboard_fusion, "ProbabilisticFusion",
              "fusion name which enabled onboard");

DEFINE_double(query_signal_range, 100.0, "max distance to front signals");
DEFINE_bool(output_raw_img, false, "write raw image to disk");
DEFINE_bool(output_debug_img, false, "write debug image to disk");

/// Temporarily change Kalman motion fusion to config here.
DEFINE_double(q_matrix_coefficient_amplifier, 0.5,
              "Kalman fitler matrix Q coeffcients");
DEFINE_double(r_matrix_amplifier, 1, "Kalman fitler matrix r coeffcients");
DEFINE_double(p_matrix_amplifier, 1, "Kalman fitler matrix p coeffcients");

DEFINE_double(a_matrix_covariance_coeffcient_1, 0.05,
              "Kalman fitler matrix a coeffcients, a_matrix_(0, 2)");
DEFINE_double(a_matrix_covariance_coeffcient_2, 0.05,
              "Kalman fitler matrix a coeffcients, a_matrix_(1, 3)");

/// calibration_config_manager.cc
DEFINE_int32(obs_camera_detector_gpu, 0, "device id for camera detector");

// obstacle/onboard/lane_post_processing_subnode.cc
DEFINE_string(onboard_lane_post_processor, "CCLanePostProcessor",
              "onboard lane post-processing algorithm name");

/// visualization

DEFINE_bool(show_radar_objects, false, "");

DEFINE_bool(show_camera_objects2d, false, "");
DEFINE_bool(show_camera_objects, false, "");
DEFINE_bool(show_camera_parsing, false, "");

DEFINE_bool(show_fused_objects, false, "");

DEFINE_bool(show_fusion_association, false, "");

DEFINE_bool(capture_screen, false, "");

DEFINE_string(screen_output_dir, "./", "");

DEFINE_string(frame_visualizer, "GLFusionVisualizer", "");

DEFINE_bool(async_fusion, false, "use distance angle ");
DEFINE_bool(use_distance_angle_fusion, true,
            "use distance angle prob distance in fusion");
DEFINE_bool(publish_fusion_event, false, "publish fusion event");
DEFINE_bool(bag_mode, false, "run perception in bag mode");

DEFINE_bool(show_motion, false, "visualize motion and object trajectories");
DEFINE_bool(skip_camera_frame, false, "skip camera frame");
DEFINE_int32(camera_hz, 30, "camera hz");
DEFINE_string(fusion_publish_sensor_id, "velodyne_64", "fusion publish id");

DEFINE_int32(pbf_fusion_assoc_distance_percent, 20, "fusion distance percent");
DEFINE_double(pbf_distance_speed_cos_diff, 0.5, "fusion velocity cosine diff");

DEFINE_string(cc_lane_post_processor_config_file,
              "modules/perception/model/camera/lane_post_process_config.pb.txt",
              "The config file of cc_lane_post_processor.");
DEFINE_string(probabilistic_fusion_config_file,
              "modules/perception/model/probabilistic_fusion_config.pb.txt",
              "The config file of probabilistic_fusion.");
DEFINE_string(yolo_config_filename, "config.pt", "Yolo config filename.");
DEFINE_string(
    yolo_camera_detector_config,
    "modules/perception/model/camera/yolo_camera_detector_config.pb.txt",
    "Yolo camera detector config filename.");
DEFINE_string(modest_radar_detector_config,
              "modules/perception/model/modest_radar_detector_config.pb.txt",
              "modest radar detector config filename.");
