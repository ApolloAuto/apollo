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

#include "modules/perception/common/base/object.h"

namespace apollo {
namespace perception {
namespace base {

Object::Object() {
  center_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  velocity_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  acceleration_uncertainty << 0.0f, 0, 0, 0, 0.0f, 0, 0, 0, 0.0f;
  type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  sub_type_probs.resize(static_cast<int>(ObjectSubType::MAX_OBJECT_TYPE), 0.0f);
  b_cipv = false;
//  feature.reset();
}

void Object::Reset() {
  id = -1;
  polygon.clear();
  direction = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  theta = 0.0f;
  theta_variance = 0.0f;
  center = Eigen::Vector3d(0.0, 0.0, 0.0);
  center_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

  size = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  size_variance = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  anchor_point = Eigen::Vector3d(0.0, 0.0, 0.0);
  type = ObjectType::UNKNOWN;
  type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0.0f);
  sub_type = ObjectSubType::UNKNOWN;
  sub_type_probs.assign(static_cast<int>(ObjectSubType::MAX_OBJECT_TYPE), 0.0f);

  confidence = 1.0f;

  is_front_critical = false;

  track_id = -1;
  velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  velocity_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  velocity_converged = true;
  velocity_confidence = 1.0;
  acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  acceleration_uncertainty << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f;

  tracking_time = 0.0;
  latest_tracked_time = 0.0;

  car_light.Reset();
  motion_state = MotionState::UNKNOWN;

  lidar_supplement.Reset();
  radar_supplement.Reset();
  radar4d_supplement.Reset();
  camera_supplement.Reset();
  fusion_supplement.Reset();
//  feature.reset();
}

std::string Object::ToString() const {
  std::ostringstream oss;
  oss << "Object [id: " << id << ", track_id: " << track_id << ", direction: ("
      << direction[0] << "," << direction[1] << "," << direction[2]
      << "), theta: " << theta << ", theta_variance: " << theta_variance
      << ", center: (" << center[0] << "," << center[1] << "," << center[2]
      << ")"
      << ", center_uncertainty: (" << center_uncertainty(0, 0) << ","
      << center_uncertainty(0, 1) << "," << center_uncertainty(0, 2) << ","
      << center_uncertainty(1, 0) << "," << center_uncertainty(1, 1) << ","
      << center_uncertainty(1, 2) << "," << center_uncertainty(2, 0) << ","
      << center_uncertainty(2, 1) << "," << center_uncertainty(2, 2)
      << "), size: (" << size[0] << "," << size[1] << "," << size[2]
      << "), size_variance: (" << size_variance[0] << "," << size_variance[1]
      << "," << size_variance[2] << "), anchor_point: (" << anchor_point[0]
      << "," << anchor_point[1] << "," << anchor_point[2]
      << "), type: " << static_cast<int>(type) << ", confidence: " << confidence
      << ", track_id: " << track_id << ", velocity: (" << velocity[0] << ","
      << velocity[1] << "," << velocity[2]
      << "), velocity_confidence: " << velocity_confidence
      << ", acceleration: (" << acceleration[0] << "," << acceleration[1] << ","
      << acceleration[2] << "), tracking_time: " << tracking_time
      << ", latest_tracked_time: " << latest_tracked_time;
  return oss.str();
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
