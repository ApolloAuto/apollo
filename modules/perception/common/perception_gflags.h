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

#ifndef MODULES_PERCEPTION_COMMON_PERCEPTION_GFLAGS_H_
#define MODULES_PERCEPTION_COMMON_PERCEPTION_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(perception_adapter_config_filename);

/// lib/config_manager/config_manager.cc
DECLARE_string(config_manager_path);
DECLARE_string(work_root);

/// obstacle/base/object.cc
DECLARE_bool(is_serialize_point_cloud);

/// obstacle/onboard/hdmap_input.cc
DECLARE_double(map_radius);
DECLARE_int32(map_sample_step);

/// obstacle/onboard/lidar_process.cc
DECLARE_bool(enable_hdmap_input);
DECLARE_string(onboard_roi_filter);
DECLARE_string(onboard_segmentor);
DECLARE_string(onboard_object_builder);
DECLARE_string(onboard_tracker);
DECLARE_int32(tf2_buff_in_ms);
DECLARE_string(lidar_tf2_frame_id);
DECLARE_string(lidar_tf2_child_frame_id);
DECLARE_string(obstacle_module_name);
DECLARE_bool(enable_visualization);

#endif /* MODULES_PERCEPTION_COMMON_PERCEPTION_GFLAGS_H_ */
