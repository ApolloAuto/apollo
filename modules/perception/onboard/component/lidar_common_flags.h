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
#ifndef PERCEPTION_ONBOARD_COMPONENT_LIDAR_COMMON_FLAGS_H_
#define PERCEPTION_ONBOARD_COMPONENT_LIDAR_COMMON_FLAGS_H_

#include <gflags/gflags.h>

namespace apollo {
namespace perception {
namespace onboard {

DECLARE_double(obs_velodyne64_query_tf_offset);

DECLARE_string(obs_lidar2novatel_tf2_child_frame_id);

DECLARE_string(obs_segmentation_component_output_channel_name);

DECLARE_string(obs_recognition_component_output_channel_name);

DECLARE_string(obs_lidar_onboard_sensor_name);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_ONBOARD_COMPONENT_LIDAR_COMMON_FLAGS_H_
