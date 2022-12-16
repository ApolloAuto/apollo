/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

namespace apollo {
namespace perception {

// sensor_manager
DECLARE_string(obs_sensor_intrinsic_path);
DECLARE_string(obs_sensor_meta_path);

DECLARE_bool(enable_base_object_pool);

// config_manager
DECLARE_string(config_manager_path);
DECLARE_string(work_root);

// lidar_point_pillars
DECLARE_int32(gpu_id);
DECLARE_string(pfe_torch_file);
DECLARE_string(scattered_torch_file);
DECLARE_string(backbone_torch_file);
DECLARE_string(fpn_torch_file);
DECLARE_string(bbox_head_torch_file);
DECLARE_double(normalizing_factor);
DECLARE_int32(num_point_feature);
DECLARE_bool(enable_ground_removal);
DECLARE_double(ground_removal_height);
DECLARE_bool(enable_downsample_beams);
DECLARE_int32(downsample_beams_factor);
DECLARE_bool(enable_downsample_pointcloud);
DECLARE_double(downsample_voxel_size_x);
DECLARE_double(downsample_voxel_size_y);
DECLARE_double(downsample_voxel_size_z);
DECLARE_bool(enable_fuse_frames);
DECLARE_int32(num_fuse_frames);
DECLARE_double(fuse_time_interval);
DECLARE_bool(enable_shuffle_points);
DECLARE_int32(max_num_points);
DECLARE_bool(reproduce_result_mode);
DECLARE_double(score_threshold);
DECLARE_double(nms_overlap_threshold);
DECLARE_int32(num_output_box_feature);

// lidar_mask_pillars
DECLARE_string(mask_pfe_torch_file);
DECLARE_string(mask_scattered_torch_file);
DECLARE_string(mask_backbone_torch_file);
DECLARE_string(mask_fpn_torch_file);
DECLARE_string(mask_bbox_head_torch_file);

// lidar_center_point
DECLARE_string(center_point_model_file);
DECLARE_string(center_point_params_file);
DECLARE_bool(use_trt);
DECLARE_int32(trt_precision);
DECLARE_int32(trt_use_static);
DECLARE_string(trt_static_dir);
DECLARE_int32(collect_shape_info);
DECLARE_string(dynamic_shape_file);

// bev petr_v1
DECLARE_string(bev_model_file);
DECLARE_string(bev_params_file);

// caddn
DECLARE_string(caddn_model_file);
DECLARE_string(caddn_params_file);

// emergency detection onnx
DECLARE_string(onnx_obstacle_detector_model);
DECLARE_string(onnx_test_input_path);
DECLARE_string(onnx_test_input_name_file);
DECLARE_string(onnx_prediction_image_path);
DECLARE_int32(num_classes);

// emergency detection libtorch
DECLARE_string(torch_detector_model);

// lidar sensor name
DECLARE_string(lidar_sensor_name);
}  // namespace perception
}  // namespace apollo
