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

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "modules/perception/common/base/camera.h"

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
                           Eigen::Vector2d* point_out);

// @brief: estimate whether two cameras have field overlap.
bool IsCamerasFieldOverlap(const base::PinholeCameraModel& from_camera,
                           const base::PinholeCameraModel& to_camera,
                           const Eigen::Matrix4d& extrinsic,
                           Eigen::Vector2d* up_left,
                           Eigen::Vector2d* low_right);

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
