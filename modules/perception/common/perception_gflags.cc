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
DEFINE_string(work_root, "modules/perception", "Project work root direcotry.");

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

DEFINE_int32(tf2_buff_in_ms, 10, "the tf2 buff size in ms");
DEFINE_int32(localization_buffer_size, 40, "localization buffer size");
DEFINE_string(lidar_tf2_frame_id, "novatel", "the tf2 transform frame id");
DEFINE_string(lidar_tf2_child_frame_id, "velodyne64",
              "the tf2 transform child frame id");
DEFINE_string(obstacle_module_name, "perception_obstacle",
              "perception obstacle module name");
DEFINE_bool(enable_visualization, false, "enable visualization for debug");

/// obstacle/perception.cc
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

/// obstacle/onboard/fusion_subnode.cc
DEFINE_string(onboard_fusion, "ProbabilisticFusion",
              "fusion name which enabled onboard");

DEFINE_double(query_signal_range, 100.0, "max distance to front signals");
DEFINE_bool(output_raw_img, false, "write raw image to disk");
DEFINE_bool(output_debug_img, false, "write debug image to disk");
