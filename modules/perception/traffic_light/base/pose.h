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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H

#include <eigen3/Eigen/Core>

namespace apollo {
namespace perception {
namespace traffic_light {

// @brief Car's Pose
class CarPose {
 public:
  CarPose() = default;

  virtual ~CarPose() = default;

  bool set_pose(const Eigen::Matrix4d &pose);

  const Eigen::Matrix4d pose() const;

 private:
  Eigen::Matrix4d _pose;

  friend std::ostream &operator<<(std::ostream &os, const CarPose &);
};

std::ostream &operator<<(std::ostream &os, const CarPose &pose);

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H
// @date 2016/09/08 17:48:06
