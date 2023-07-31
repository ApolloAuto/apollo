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
#include "modules/perception/common/algorithm/geometry/camera_homography.h"

#include <limits>

namespace apollo {
namespace perception {
namespace algorithm {

// @brief: find the corresponding 2D point in camera2, given 2D point in
// camera1. assume the two cameras have the same optical center.
// notice camera1_intrinsic_inverse is inverse version
bool PointCamera1ToCamera2(const Eigen::Vector2d& point,
                           const Eigen::Matrix3d& camera1_intrinsic_inverse,
                           const Eigen::Matrix3d& camera2_intrinsic,
                           const Eigen::Matrix3d& trans_camera1_to_camera2,
                           Eigen::Vector2d* point_out) {
  Eigen::Vector3d pt = point.homogeneous();
  Eigen::Vector3d camera2_3d =
      static_cast<Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
          trans_camera1_to_camera2 * camera1_intrinsic_inverse * pt);

  double z = camera2_3d(2);
  if (fabs(z) > 1e-6) {
    *point_out = (camera2_intrinsic * camera2_3d / z).head(2);
    return true;
  }
  return false;
}

// @brief: estimate whether two cameras have field overlap.
bool IsCamerasFieldOverlap(const base::PinholeCameraModel& from_camera,
                           const base::PinholeCameraModel& to_camera,
                           const Eigen::Matrix4d& extrinsic,
                           Eigen::Vector2d* up_left,
                           Eigen::Vector2d* low_right) {
  size_t f_width = from_camera.get_width();
  size_t f_height = from_camera.get_height();
  size_t to_width = to_camera.get_width();
  size_t to_height = to_camera.get_height();
  Eigen::Matrix3d f2t_rotation = extrinsic.topLeftCorner(3, 3);
  Eigen::Matrix<double, 4, 2> points;
  points << 0.0, 0.0, static_cast<double>(f_width), 0.0,
      static_cast<double>(f_width), static_cast<double>(f_height), 0.0,
      static_cast<double>(f_height);
  Eigen::Vector2d pt_min(std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
  Eigen::Vector2d pt_max(-std::numeric_limits<double>::max(),
                         -std::numeric_limits<double>::max());
  Eigen::Matrix3d from_camera_intrinsic_inverse =
      from_camera.get_intrinsic_params().cast<double>().inverse();
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector2d point_out;
    bool status = PointCamera1ToCamera2(
        points.block<1, 2>(i, 0), from_camera_intrinsic_inverse,
        to_camera.get_intrinsic_params().cast<double>(), f2t_rotation,
        &point_out);
    if (status) {
      pt_min = pt_min.cwiseMin(point_out);
      pt_max = pt_max.cwiseMax(point_out);
    }
  }
  (*up_left) = pt_min.cwiseMax(Eigen::Vector2d(0, 0));
  (*low_right) = pt_max.cwiseMin(Eigen::Vector2d(to_width, to_height));
  if ((up_left->array() < low_right->array()).all()) {
    return true;
  }
  return false;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
