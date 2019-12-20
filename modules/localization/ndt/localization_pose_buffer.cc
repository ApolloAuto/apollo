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

#include "modules/localization/ndt/localization_pose_buffer.h"

#include <iomanip>

#include "Eigen/Dense"

#include "cyber/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"

namespace apollo {
namespace localization {
namespace ndt {

const unsigned int LocalizationPoseBuffer::s_buffer_size_ = 20;

LocalizationPoseBuffer::LocalizationPoseBuffer() {
  lidar_poses_.resize(s_buffer_size_);
  used_buffer_size_ = 0;
  head_index_ = 0;
  has_initialized_ = false;
}

LocalizationPoseBuffer::~LocalizationPoseBuffer() {}

void LocalizationPoseBuffer::UpdateLidarPose(
    double timestamp, const Eigen::Affine3d& locator_pose,
    const Eigen::Affine3d& novatel_pose) {
  if (!has_initialized_) {
    lidar_poses_[head_index_].locator_pose = locator_pose;
    lidar_poses_[head_index_].locator_pose.linear() = novatel_pose.linear();
    lidar_poses_[head_index_].novatel_pose = novatel_pose;
    lidar_poses_[head_index_].timestamp = timestamp;
    ++used_buffer_size_;
    has_initialized_ = true;
  } else {
    // add 10Hz pose
    unsigned int empty_position =
        (head_index_ + used_buffer_size_) % s_buffer_size_;
    lidar_poses_[empty_position].locator_pose = locator_pose;
    lidar_poses_[empty_position].novatel_pose = novatel_pose;
    lidar_poses_[empty_position].timestamp = timestamp;

    if (used_buffer_size_ == s_buffer_size_) {
      head_index_ = (head_index_ + 1) % s_buffer_size_;
    } else {
      ++used_buffer_size_;
    }
  }
}

Eigen::Affine3d LocalizationPoseBuffer::UpdateOdometryPose(
    double timestamp, const Eigen::Affine3d& novatel_pose) {
  Eigen::Affine3d pose = novatel_pose;
  if (used_buffer_size_ > 0) {
    pose.translation()[0] = 0;
    pose.translation()[1] = 0;
    pose.translation()[2] = 0;
    Eigen::Affine3d predict_pose;
    double weight = 0.0;

    Eigen::Quaterniond novatel_quat(novatel_pose.linear());
    novatel_quat.normalize();
    common::math::EulerAnglesZXYd novatel_ev(
        novatel_quat.w(), novatel_quat.x(), novatel_quat.y(), novatel_quat.z());
    Eigen::Vector3d pose_ev;
    pose_ev[0] = 0;
    pose_ev[1] = 0;
    pose_ev[2] = 0;

    for (unsigned int c = 0, i = head_index_; c < used_buffer_size_;
         ++c, i = (i + 1) % s_buffer_size_) {
      predict_pose.translation() = lidar_poses_[i].locator_pose.translation() -
                                   lidar_poses_[i].novatel_pose.translation() +
                                   novatel_pose.translation();
      pose.translation() += predict_pose.translation();

      Eigen::Quaterniond pair_locator_quat(
          lidar_poses_[i].locator_pose.linear());
      pair_locator_quat.normalize();
      Eigen::Quaterniond pair_novatel_quat(
          lidar_poses_[i].novatel_pose.linear());
      pair_novatel_quat.normalize();
      Eigen::Quaterniond predict_pose_quat =
          pair_locator_quat * pair_novatel_quat.inverse() * novatel_quat;
      predict_pose_quat.normalized();

      common::math::EulerAnglesZXYd predict_pose_ev(
          predict_pose_quat.w(), predict_pose_quat.x(), predict_pose_quat.y(),
          predict_pose_quat.z());
      double predict_yaw = predict_pose_ev.yaw();
      if (novatel_ev.yaw() > 0) {
        if (novatel_ev.yaw() - predict_pose_ev.yaw() > M_PI) {
          predict_yaw = predict_pose_ev.yaw() + M_PI * 2.0;
        }
      } else {
        if (predict_pose_ev.yaw() - novatel_ev.yaw() > M_PI) {
          predict_yaw = predict_pose_ev.yaw() - M_PI * 2.0;
        }
      }
      pose_ev[2] += predict_yaw;
      weight += 1;
    }
    pose.translation() *= (1.0 / weight);
    pose_ev[0] = novatel_ev.pitch();
    pose_ev[1] = novatel_ev.roll();
    pose_ev[2] *= (1.0 / weight);
    if (std::abs(pose_ev[2]) > M_PI) {
      if (pose_ev[2] > 0) {
        pose_ev[2] -= 2.0 * M_PI;
      } else {
        pose_ev[2] += 2.0 * M_PI;
      }
    }
    common::math::EulerAnglesZXYd pose_euler(pose_ev[1], pose_ev[0],
                                             pose_ev[2]);
    Eigen::Quaterniond tmp_qbn = pose_euler.ToQuaternion();
    tmp_qbn.normalize();
    pose.linear() = tmp_qbn.toRotationMatrix();
  }
  return pose;
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
