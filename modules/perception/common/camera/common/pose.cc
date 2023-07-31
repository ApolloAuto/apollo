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
#include "modules/perception/common/camera/common/pose.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

bool CarPose::Init(double ts, const Eigen::Matrix4d &pose) {
  timestamp_ = ts;
  pose_ = pose;
  return true;
}

const Eigen::Matrix4d CarPose::getCarPose() const { return pose_; }

const Eigen::Vector3d CarPose::getCarPosition() const {
  Eigen::Vector3d p;
  p[0] = pose_(0, 3);
  p[1] = pose_(1, 3);
  p[2] = pose_(2, 3);

  return p;
}

void CarPose::SetCameraPose(const std::string &camera_name,
                            const Eigen::Matrix4d &c2w_pose) {
  c2w_poses_[camera_name] = c2w_pose;
}

bool CarPose::GetCameraPose(const std::string &camera_name,
                            Eigen::Matrix4d *c2w_pose) const {
  if (c2w_pose == nullptr) {
    AERROR << "c2w_pose is not available";
    return false;
  }
  if (c2w_poses_.find(camera_name) == c2w_poses_.end()) {
    return false;
  }
  *c2w_pose = c2w_poses_.at(camera_name);

  return true;
}

std::ostream &operator<<(std::ostream &os, const CarPose &pose) {
  os << pose.pose_;
  return os;
}
void CarPose::ClearCameraPose(const std::string &camera_name) {
  auto it = c2w_poses_.find(camera_name);
  if (it != c2w_poses_.end()) {
    c2w_poses_.erase(it);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
