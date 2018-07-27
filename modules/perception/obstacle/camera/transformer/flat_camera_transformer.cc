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

  // Center in Camera Space and Derived Center in Ego Car Space
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> centers;

  for (auto obj_ptr : *objects) {
    // Get flat 2D distance in meter
    float d = obj_ptr->distance;
    float d_v = std::abs(camera2car_(2, 3) - obj_ptr->height / 2.0f);
    float d_flat = sqrt(d * d - d_v * d_v);

    // Get 2D vector of top down view
    Eigen::Vector3f center = obj_ptr->center;  // Center in Camera Space
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

    if (HaveHighConfidence(obj_ptr)) {
      Eigen::Vector3f center_ego = pos_ground
       + Eigen::Vector3f(0.0f, 0.0f, obj_ptr->height / 2.0f);
      centers.emplace_back(std::make_pair(center, center_ego));
    }
  }

  GetDynamicExtrinsics(centers);
  return true;
}

bool FlatCameraTransformer::SetExtrinsics(
    const Eigen::Matrix<double, 4, 4> &extrinsics) {
  camera2car_ = extrinsics.cast<float>();
  camera2car_adj_ = camera2car_;
  camera2car_flat_offset_ =
      Eigen::Matrix<float, 3, 1>(camera2car_(0, 3), camera2car_(1, 3), 0.0f);
  return true;
}

bool FlatCameraTransformer::GetAdjustedExtrinsics(
  Eigen::Matrix<double, 4, 4>* extrinsics) {
  // Return static results if no object to use in the scene
  if (adjust_pitch_) {
    *extrinsics = camera2car_adj_.cast<double>();
  } else {
    *extrinsics = camera2car_.cast<double>();
  }
  return adjust_pitch_;
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

bool FlatCameraTransformer::HaveHighConfidence(
  std::shared_ptr<VisualObject> obj_ptr) {
    if (obj_ptr->trunc_width > 0.25f) return false;
    if (obj_ptr->trunc_height > 0.25f) return false;
    if (obj_ptr->distance > 50.0) return false;
    if (obj_ptr->distance < 5.0) return false;

    double azimuth = std::atan2(obj_ptr->center[1], obj_ptr->center[0])
     * 180.0 / M_PI;
    if (!(-30.0 < azimuth && azimuth < 30.0)) return false;

    return true;
}

void FlatCameraTransformer::GetDynamicExtrinsics(
  const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &centers) {
    if (centers.empty()) {
      adjust_pitch_ = false;
      camera2car_adj_ = camera2car_;
      pitch_diff_ = 0.0f;
      return;
    }

    float min_diff = std::numeric_limits<float>::max();
    float best_pitch_adjustment = 0.0f;
    Eigen::Matrix<float, 4, 4> best_camera2car = camera2car_;

    Eigen::Matrix<float, 4, 4> rot;
    rot.setIdentity();
    for (float p = -3.0; p < 3.0; p += 0.2f) {
      // Create adjusted extrinsics
      Eigen::Matrix3f rotate(Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ())
      * Eigen::AngleAxisf(p / 180.0f * M_PI, Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX()));
      rot.block<3, 3>(0, 0) = rotate;
      Eigen::Matrix4f camera2car_r = rot * camera2car_;

      float diff = 0.0;
      for (auto center_p : centers) {
        auto c_cam = center_p.first;
        auto c_car_t = center_p.second;

        Eigen::Vector4f c(c_cam.x(), c_cam.y(), c_cam.z(), 1.0f);
        Eigen::Vector4f c_car_d = camera2car_r * c;

        auto diff_3dim = c_car_d.head(3) - c_car_t;
        diff += std::sqrt(diff_3dim.x() * diff_3dim.x()
                          + diff_3dim.y() * diff_3dim.y()
                          + diff_3dim.z() * diff_3dim.z());
      }

      // Get best pitch angle adjustment
      if (diff < min_diff) {
        min_diff = diff;
        best_pitch_adjustment = p;
        best_camera2car = camera2car_r;
      }
    }

    // Output extrinsics as result
    adjust_pitch_ = true;
    camera2car_adj_ = best_camera2car;
    pitch_diff_ = best_pitch_adjustment;
}

}  // namespace perception
}  // namespace apollo
