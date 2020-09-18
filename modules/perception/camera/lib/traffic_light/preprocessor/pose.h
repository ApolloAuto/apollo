/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <map>
#include <string>

#include "Eigen/Core"

#include "modules/common/util/eigen_defs.h"
#include "modules/perception/base/image_8u.h"

namespace apollo {
namespace perception {
namespace camera {

// @brief Car's Pose
class CarPose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  CarPose() = default;
  ~CarPose() = default;

  bool Init(double ts, const Eigen::Matrix4d& pose);

  const Eigen::Matrix4d getCarPose() const;
  const Eigen::Vector3d getCarPosition() const;

  void ClearCameraPose(const std::string& camera_name);
  void SetCameraPose(const std::string& camera_name,
                     const Eigen::Matrix4d& c2w_pose);
  bool GetCameraPose(const std::string& camera_name,
                     Eigen::Matrix4d* c2w_pose) const;

  void setTimestamp(double ts) { timestamp_ = ts; }
  double getTimestamp() const { return timestamp_; }

  Eigen::Matrix4d pose_;  // car(novatel) to world pose
  // camera to world poses
  apollo::common::EigenMap<std::string, Eigen::Matrix4d> c2w_poses_;
  double timestamp_;

 private:
  friend std::ostream& operator<<(std::ostream& os, const CarPose&);
};

std::ostream& operator<<(std::ostream& os, const CarPose& pose);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
