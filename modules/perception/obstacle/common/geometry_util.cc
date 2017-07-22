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
#include "modules/perception/obstacle/common/geometry_util.h"

namespace apollo {
namespace perception {

void TransAffineToMatrix4(
    const Eigen::Vector3d& translation,
    const Eigen::Vector4d& rotation,
    Eigen::Matrix4d* trans_matrix) {
  const double t_x = translation(0);
  const double t_y = translation(1);
  const double t_z = translation(2);

  const double qx = rotation(0);
  const double qy = rotation(1);
  const double qz = rotation(2);
  const double qw = rotation(3);

  (*trans_matrix) << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
                     2 * (qx * qz + qy * qw), t_x,
                     2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
                     2 * (qy * qz - qx * qw), t_y,
                     2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
                     1 - 2 * (qx * qx + qy * qy), t_z,
                     0, 0, 0, 1;
}

}  // namespace perception
}  // namespace apollo
