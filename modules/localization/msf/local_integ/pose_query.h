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

/**
 * @file pose_query.h
 * @brief The class of PoseQuery
 */

#ifndef MODULES_LOCALIZATION_MSF_POSE_QUERY_H_
#define MODULES_LOCALIZATION_MSF_POSE_QUERY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <iostream>
#include "modules/common/log.h"

namespace apollo {
namespace localization {

typedef Eigen::Affine3d TransformD;
typedef Eigen::Quaterniond QuaternionD;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Matrix3d MatrixDcm;

struct PoseForQuery {
 public:
  PoseForQuery() : measure_time(0),
                   translation(Vector3D::Zero()),
                   quaternion(1, 0, 0, 0) {}

  PoseForQuery(double time, const Vector3D &trans, const QuaternionD &quat)
      : measure_time(time), translation(trans), quaternion(quat) {}

  double measure_time;
  Vector3D translation;
  QuaternionD quaternion;
};

class PoseQuery {
 public:
  PoseQuery() : buffer_size_(0), buffer_max_size_(400) {}

  ~PoseQuery() {}

  void AddPose(double time, const Vector3D &trans, const QuaternionD &quat) {
    std::unique_lock<std::mutex> lock(pose_mutex_);

    if (time < pose_buffer_.back().measure_time + 1e-6) {
      return;
    }

    PoseForQuery pose(time, trans, quat);
    pose_buffer_.push_back(pose);
    ++buffer_size_;
    if (buffer_size_ > buffer_max_size_) {
      pose_buffer_.pop_front();
      --buffer_size_;
    }
    return;
  }

  bool QueryQuaternion(double time, QuaternionD *quat) {
    std::unique_lock<std::mutex> lock(pose_mutex_);

    if (buffer_size_ < 2) {
      return false;
    }

    auto itr_last = pose_buffer_.begin();
    auto itr = itr_last;
    ++itr;
    auto itr_end = pose_buffer_.end();

    // query time is too old
    if (time < itr_last->measure_time) {
      LOG(WARNING) << std::setprecision(20) 
                  << "query time is too old, query time: " << time;
      return false;
    }

    // query time is OK
    bool found_poses = false;
    for (; itr != itr_end; ++itr, ++itr_last) {
      double time0 = itr_last->measure_time;
      double time1 = itr->measure_time;
      if (time0 <= time && time <= time1) {
        double ratio = (time - time0) / (time1 - time0); // add_pose avoid /0
        *quat = itr_last->quaternion.slerp(ratio, itr->quaternion);
        return true;
      }
    }

    // query time is too new
    if (time < itr_last->measure_time + 0.02) {
        *quat = itr_last->quaternion;
    }

    LOG(WARNING) << std::setprecision(20) 
                 << "query time is too new, query time: " << time;
    return false;
  }

protected:
    std::list<PoseForQuery> pose_buffer_;
    int buffer_size_;
    int buffer_max_size_;
    std::mutex pose_mutex_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_POSE_QUERY_H_
