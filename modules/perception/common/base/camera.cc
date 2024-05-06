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

#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace base {

Eigen::Vector2f PinholeCameraModel::Project(const Eigen::Vector3f& point3d) {
  Eigen::Vector2f pt2d;

  pt2d(0) = point3d(0) / point3d(2) * intrinsic_params_(0, 0) +
            intrinsic_params_(0, 2);
  pt2d(1) = point3d(1) / point3d(2) * intrinsic_params_(1, 1) +
            intrinsic_params_(1, 2);

  return pt2d;
}

Eigen::Vector3f PinholeCameraModel::UnProject(const Eigen::Vector2f& point2d) {
  Eigen::Vector3f pt3d;
  pt3d(0) = (point2d(0) - intrinsic_params_(0, 2)) / intrinsic_params_(0, 0);
  pt3d(1) = (point2d(1) - intrinsic_params_(1, 2)) / intrinsic_params_(1, 1);
  pt3d(2) = 1.0f;

  return pt3d;
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
