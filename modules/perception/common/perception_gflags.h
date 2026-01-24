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
DECLARE_string(obs_sensor_meta_file);

DECLARE_bool(enable_base_object_pool);

// config_manager
DECLARE_string(config_manager_path);
DECLARE_string(work_root);

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

// lidar_center_point
DECLARE_bool(use_trt);
DECLARE_int32(trt_precision);
DECLARE_int32(trt_use_static);
DECLARE_bool(use_calibration);
DECLARE_bool(use_dynamicshape);
DECLARE_bool(collect_shape_info);
DECLARE_string(dynamic_shape_file);

DECLARE_string(object_template_file);

DECLARE_int32(hdmap_sample_step);

// scene manager
DECLARE_string(scene_manager_file);
DECLARE_string(roi_service_file);
DECLARE_string(ground_service_file);

// front critical judgement (in NOVATEL system)
DECLARE_bool(need_judge_front_critical);
DECLARE_double(x_front);
DECLARE_double(x_back);
DECLARE_double(y_front);
DECLARE_double(y_back);
DECLARE_double(max_points_height);

// blind TrafficCone (in NOVATEL system)
DECLARE_bool(need_reserve_blind_cone);
DECLARE_double(cone_x_front);
DECLARE_double(cone_x_back);
DECLARE_double(cone_y_front);
DECLARE_double(cone_y_back);
DECLARE_double(cone_reserve_time);

}  // namespace perception
}  // namespace apollo
