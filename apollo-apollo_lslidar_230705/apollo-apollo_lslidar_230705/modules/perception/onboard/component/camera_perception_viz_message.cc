/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/onboard/component/camera_perception_viz_message.h"

namespace apollo {
namespace perception {
namespace onboard {

CameraPerceptionVizMessage::CameraPerceptionVizMessage(
    const std::string& camera_name, const double msg_timestamp,
    const Eigen::Matrix4d& pose_camera_to_world,
    const std::shared_ptr<base::Blob<uint8_t>>& image_blob,
    const std::vector<base::ObjectPtr>& camera_objects,
    const std::vector<base::LaneLine>& lane_objects,
    const apollo::common::ErrorCode& error_code)
    : camera_name_(camera_name),
      msg_timestamp_(msg_timestamp),
      pose_camera_to_world_(pose_camera_to_world),
      image_blob_(image_blob),
      error_code_(error_code) {
  camera_objects_.clear();
  for (const auto& obj : camera_objects) {
    camera_objects_.push_back(obj);
  }
  lane_objects_.clear();
  for (const auto& obj : lane_objects) {
    lane_objects_.push_back(obj);
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
