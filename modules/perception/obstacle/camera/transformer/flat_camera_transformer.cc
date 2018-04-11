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

#include "modules/perception/obstacle/camera/transformer/flat_camera_transformer.h"

namespace apollo {
namespace perception {

bool FlatCameraTransformer::Init() { return true; }

bool FlatCameraTransformer::Transform(
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  if (!objects) return false;

  for (auto obj_ptr : *objects) {
    // Get flat 2D distance in meter
    float d = obj_ptr->distance;
    float d_v = std::abs(camera2car_(2, 3) - obj_ptr->height / 2.0f);
    float d_flat = sqrt(d * d - d_v * d_v);

    // Get 2D vector of top down view
    Eigen::Vector3f center = obj_ptr->center;
    Eigen::Vector4f center_v(center.x(), center.y(), center.z(), 0.0f);
    center_v = camera2car_ * center_v;
    center_v.z() = 0.0f;
    Eigen::Vector3f unit_v_flat = MakeUnit(center_v.head(3));

    // 2D position in top-down view of ego-car space
    Eigen::Vector3f pos_ground = unit_v_flat * d_flat;
    obj_ptr->center = pos_ground + camera2car_flat_offset_;

    // Orientation
    // Camera space
    float theta = obj_ptr->theta;
    Eigen::Vector4f dir(cos(theta), 0.0f, -sin(theta), 0.0f);

    // Ego car space
    dir = camera2car_ * dir;
    obj_ptr->direction = dir.head(3);
    obj_ptr->theta = atan2(dir[1], dir[0]);
  }

  return true;
}

bool FlatCameraTransformer::SetExtrinsics(
    const Eigen::Matrix<double, 4, 4> &extrinsics) {
  camera2car_ = extrinsics.cast<float>();
  camera2car_flat_offset_ =
      Eigen::Matrix<float, 3, 1>(camera2car_(0, 3), camera2car_(1, 3), 0.0f);
  return true;
}

std::string FlatCameraTransformer::Name() const {
  return "FlatCameraTransformer";
}

Eigen::Matrix<float, 3, 1> FlatCameraTransformer::MakeUnit(
    const Eigen::Matrix<float, 3, 1> &v) const {
  Eigen::Matrix<float, 3, 1> unit_v = v;
  float to_unit_scale =
      std::sqrt(unit_v.x() * unit_v.x() + unit_v.y() * unit_v.y() +
                unit_v.z() * unit_v.z());
  unit_v /= to_unit_scale;
  return unit_v;
}

}  // namespace perception
}  // namespace apollo
