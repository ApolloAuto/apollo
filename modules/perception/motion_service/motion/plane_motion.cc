/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/motion_service/motion/plane_motion.h"

#include <limits>
#include <list>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

PlaneMotion::PlaneMotion(int s) {
  set_buffer_size(s);
  if (mat_motion_sensor_.rows() == 3 && mat_motion_sensor_.cols() == 3) {
    is_3d_motion_ = false;
  } else if (mat_motion_sensor_.rows() == 4 && mat_motion_sensor_.cols() == 4) {
    is_3d_motion_ = true;
  } else {
    AERROR << "Unknow motion matrix size : " << mat_motion_sensor_.rows() << " "
           << mat_motion_sensor_.cols();
  }
}

PlaneMotion::~PlaneMotion() {
  if (mot_buffer_ != nullptr) {
    mot_buffer_->clear();
    mot_buffer_ = nullptr;
  }
}

// Generate the inverse motion for past trajectory
void PlaneMotion::generate_motion_matrix(base::VehicleStatus *vehicledata) {
  float time_d = static_cast<float>(vehicledata->time_d);
  if (!is_3d_motion_) {
    base::MotionType motion_2d = base::MotionType::Identity();
    float theta = time_d * vehicledata->yaw_rate;
    Eigen::Rotation2Df rot2d(theta);
    Eigen::Vector2f trans;
    float velocity = static_cast<float>(
        sqrt(vehicledata->velocity_x * vehicledata->velocity_x +
             vehicledata->velocity_y * vehicledata->velocity_y));
    float displacement = time_d * velocity;
    trans(0) = static_cast<float>(displacement * cos(theta));
    trans(1) = static_cast<float>(displacement * sin(theta));
    // trans(0) = time_d * vehicledata->velocity_x;
    // trans(1) = time_d * vehicledata->velocity_y;

    motion_2d.block(0, 0, 2, 2) = rot2d.toRotationMatrix().transpose();
    motion_2d.block(0, 2, 2, 1) = -rot2d.toRotationMatrix().transpose() * trans;
    ACHECK(vehicledata->motion.rows() == motion_2d.rows());
    ACHECK(vehicledata->motion.cols() == motion_2d.cols());
    vehicledata->motion = motion_2d;
  } else {
    base::MotionType motion_3d = base::MotionType::Identity();
    float roll_delta = time_d * vehicledata->roll_rate;
    float pitch_delta = time_d * vehicledata->pitch_rate;
    float yaw_delta = time_d * vehicledata->yaw_rate;

    Eigen::AngleAxisf roll_angle(roll_delta, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch_delta, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw_delta, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = roll_angle * pitch_angle * yaw_angle;
    Eigen::Matrix3f rot3d = q.matrix();

    float displacement = time_d * vehicledata->velocity;
    Eigen::Vector3f trans;
    trans(0) =
        static_cast<float>(sqrt(displacement * displacement /
                                (tan(yaw_delta) * tan(yaw_delta) +
                                 tan(pitch_delta) * tan(pitch_delta) + 1)));
    trans(1) = static_cast<float>(tan(yaw_delta) * trans(0));
    trans(2) = static_cast<float>(tan(pitch_delta) * trans(0));

    motion_3d.block(0, 0, 3, 3) = rot3d.transpose();
    motion_3d.block(0, 3, 3, 1) = -rot3d.transpose() * trans;
    ACHECK(vehicledata->motion.rows() == motion_3d.rows());
    ACHECK(vehicledata->motion.cols() == motion_3d.cols());
    vehicledata->motion = motion_3d;
  }
}

void PlaneMotion::accumulate_motion(const double start_time,
                                    const double end_time) {
  // accumulate CAN+IMU / Localization motion
  auto iter = raw_motion_queue_.begin();
  for (; iter != raw_motion_queue_.end() && iter->time_ts <= end_time; ++iter) {
    if (iter->time_ts < start_time) {
      continue;
    }
    mat_motion_sensor_ *= iter->motion;
    time_difference_ += static_cast<float>(iter->time_d);
  }
  // clean raw_motion_queue useless history
  while (raw_motion_queue_.begin() != iter) {
    raw_motion_queue_.pop_front();
  }
}

void PlaneMotion::update_motion_buffer(const base::VehicleStatus &vehicledata,
                                       const double pre_image_timestamp,
                                       const double image_timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  // compute the projection from pevious frames to the last frame
  for (size_t k = 0; k < mot_buffer_->size(); ++k) {
    mot_buffer_->at(k).motion *= mat_motion_sensor_;
  }

  // set time_diff as image_time_diff
  time_difference_ = static_cast<float>(image_timestamp - pre_image_timestamp);

  // a new motion between images
  mot_buffer_->push_back(vehicledata);
  mot_buffer_->back().time_d = time_difference_;
  // update motion
  mot_buffer_->back().motion = mat_motion_sensor_;
  mot_buffer_->back().time_ts = image_timestamp;
  // reset motion buffer
  mat_motion_sensor_ =
      base::MotionType::Identity();  // reset image accumulated motion
  time_difference_ = 0.0f;           // reset the accumulated time difference
}

bool PlaneMotion::find_motion_with_timestamp(double timestamp,
                                             base::VehicleStatus *vs) {
  std::lock_guard<std::mutex> lock(mutex_);
  ADEBUG << "mot_buffer_->size(): " << mot_buffer_->size();

  for (auto rit = mot_buffer_->rbegin(); rit != mot_buffer_->rend(); ++rit) {
    if (std::abs(rit->time_ts - timestamp) <
        std::numeric_limits<double>::epsilon()) {
      *vs = *rit;
      return true;
    }
  }
  return false;
}

base::MotionBuffer PlaneMotion::get_buffer() {
  std::lock_guard<std::mutex> lock(mutex_);
  return *mot_buffer_;
}

void PlaneMotion::add_new_motion(double pre_image_timestamp,
                                 double image_timestamp,
                                 int motion_operation_flag,
                                 base::VehicleStatus *vehicledata) {
  while (!raw_motion_queue_.empty() &&
         vehicledata->time_ts < raw_motion_queue_.back().time_ts) {
    raw_motion_queue_.pop_back();
    ADEBUG << "pop ts : back ts" << vehicledata->time_ts << " "
           << raw_motion_queue_.back().time_ts << " "
           << raw_motion_queue_.size();
  }

  if (motion_operation_flag != RESET) {
    generate_motion_matrix(vehicledata);
    raw_motion_queue_.push_back(*vehicledata);
    if (static_cast<int>(raw_motion_queue_.size()) > buffer_size_ * 10) {
      AWARN << "MotionQueue is too large, try sync motion/image timestep";
    }

    switch (motion_operation_flag) {
      case ACCUM_MOTION:
        // do nothing
        break;
      case ACCUM_PUSH_MOTION:
        accumulate_motion(pre_image_timestamp, image_timestamp);
        update_motion_buffer(*vehicledata, pre_image_timestamp,
                             image_timestamp);
        break;
      default:
        AERROR << "motion operation flag:wrong type";
        return;
    }
  } else {
    mot_buffer_->clear();
    vehicledata->time_d = 0;
    vehicledata->time_ts = image_timestamp;
    vehicledata->motion = base::MotionType::Identity();
    mot_buffer_->push_back(*vehicledata);
    ADEBUG << "pop and rest raw_buffer, mot_buffer: "
           << raw_motion_queue_.size();
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
