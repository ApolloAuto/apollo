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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_PROJECTION_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_PROJECTION_H_

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "modules/perception/traffic_light/interface/base_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

// @brief Projection for each Camera.
class BoundaryProjection : public BaseProjection {
 public:
  bool Project(const CameraCoeffient &camera_coeffient,
               const Eigen::Matrix4d &pose,
               const apollo::hdmap::Signal &tl_info,
               Light *light) const override;

 private:
  bool ProjectPoint(const CameraCoeffient &coeffient,
                    const Eigen::Matrix4d &pose,
                    const apollo::common::Point3D &point, int *center_x,
                    int *center_y) const;

  bool ProjectPointDistort(const CameraCoeffient &coeffient,
                           const Eigen::Matrix4d &pose,
                           const apollo::common::PointENU &point, int *center_x,
                           int *center_y) const;

  Eigen::Matrix<double, 2, 1> PixelDenormalize(
      const Eigen::Matrix<double, 2, 1> &pt2d,
      const Eigen::Matrix<double, 3, 4> &camera_intrinsic,
      const Eigen::Matrix<double, 5, 1> &distort_params) const;
};

REGISTER_PROJECTION(BoundaryProjection);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_PROJECTION_H_
