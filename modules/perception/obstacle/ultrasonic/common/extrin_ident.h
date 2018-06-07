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

#ifndef MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_EXTRIN_INDENT_H_
#define MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_EXTRIN_INDENT_H_

#include <vector>
#include "Eigen/Core"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Dense"

namespace apollo {
namespace perception {

class ExtrinIdent {
 public:
  ExtrinIdent() = default;

  ExtrinIdent(const Eigen::Vector3d& position,
              const Eigen::Quaterniond& orientation,
              const int ultrasonic_id,
              const float cone_fov);

  bool operator < (const ExtrinIdent &second);

  void operator = (const ExtrinIdent &second);

 private:
  int  ultrasonic_id_;
  float cone_fov_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Transform<double, 3, Eigen::Isometry> transform_;
  std::vector<Eigen::Vector3d> upoint_arc_;
  std::vector<Eigen::Vector3d> fov_margin_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_EXTRIN_INDENT_H_
