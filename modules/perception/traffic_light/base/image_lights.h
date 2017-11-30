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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_IMAGE_LIGHTS_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_IMAGE_LIGHTS_H

#include <vector>
#include <memory>

#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"
#include "modules/perception/traffic_light/base/pose.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct ImageLights {
  std::shared_ptr<Image> image;
  CarPose pose;
  std::shared_ptr<LightPtrs> lights;
  std::shared_ptr<LightPtrs> lights_outside_image; //record the lights outside the lights.
  CameraId camera_id = UNKNOWN;
  double timestamp = 0.0;   // image's timestamp

  double preprocess_receive_timestamp = 0.0;   // timestamp when received a image
  double preprocess_send_timestamp = 0.0;  // timestamp when PreprocessSubnode pub event
  bool is_pose_valid = false;
  double diff_image_pose_ts = 0.0;  // image' timestamp minus the most recently pose's timestamp
  double diff_image_sys_ts = 0.0;  // image' timestamp system's timestamp
  size_t offset = 0;  // offset size between hdmap bbox and detection bbox
  size_t num_signals = 0;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_IMAGE_LIGHTS_H
