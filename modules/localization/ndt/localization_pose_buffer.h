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

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace apollo {
namespace localization {
namespace ndt {

struct LocalizationStampedPosePair {
  double timestamp;
  Eigen::Affine3d novatel_pose;
  Eigen::Affine3d locator_pose;
};

class LocalizationPoseBuffer {
 public:
  LocalizationPoseBuffer();
  ~LocalizationPoseBuffer();
  /**@brief receive a pair of lidar pose and
   * odometry pose which almost have the same timestame
   * */
  void UpdateLidarPose(double timestamp, const Eigen::Affine3d& locator_pose,
                       const Eigen::Affine3d& novatel_pose);
  /**@brief receive an odometry pose and
   * estimate the output pose according to last lidar pose recorded.
   * */
  Eigen::Affine3d UpdateOdometryPose(double timestamp,
                                     const Eigen::Affine3d& novatel_pose);
  /**@brief Get the used size of buffer*/
  unsigned int GetUsedBufferSize() { return used_buffer_size_; }
  /**@brief Get the current head of the buffer*/
  unsigned int GetHeadIndex() { return head_index_; }

 private:
  static const unsigned int s_buffer_size_;

 private:
  std::vector<LocalizationStampedPosePair> lidar_poses_;
  unsigned int used_buffer_size_;
  unsigned int head_index_;
  bool has_initialized_;
};

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
