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

#include <limits>
#include <list>
#include "modules/perception/obstacle/camera/motion/plane_motion.h"
#include "modules/common/log.h"
namespace apollo {
namespace perception {

PlaneMotion::PlaneMotion(int s) {
  set_buffer_size(s);
}

PlaneMotion::~PlaneMotion(void) {
  if (mot_buffer_ != nullptr) {
    mot_buffer_->clear();
    mot_buffer_ = nullptr;
  }
}

// Generate the inverse motion for past trajectory
void PlaneMotion::generate_motion_matrix(VehicleStatus *vehicledata) {
  Eigen::Matrix3f motion_2d = Eigen::Matrix3f::Identity();
  float theta = vehicledata->time_d * vehicledata->yaw_rate;
  float displacement = vehicledata->time_d * vehicledata->velocity;

  Eigen::Rotation2Df rot2d(theta);
  Eigen::Vector2f trans;

  trans(0) = displacement * cos(theta);
  trans(1) = displacement * sin(theta);

  motion_2d.block(0, 0, 2, 2) = rot2d.toRotationMatrix().transpose();
  motion_2d.block(0, 2, 2, 1) = -rot2d.toRotationMatrix().transpose() * trans;

  vehicledata->motion = motion_2d;
}

void PlaneMotion::accumulate_motion(double start_time, double end_time) {
  std::list<VehicleStatus>::iterator iter_1 = raw_motion_queue_.begin();
  // locate starting motion
  while (iter_1 != raw_motion_queue_.end()
        && iter_1->time_ts < start_time) {
    iter_1++;  // iter_1 : ts >= start_time
  }
  // locate ending motion
  std::list<VehicleStatus>::iterator iter_2 = iter_1;
  while (iter_2 != raw_motion_queue_.end()
        && iter_2->time_ts <= end_time) {
    iter_2++;  // iter_2: ts > end_time
  }
  // accumulate CAN+IMU / Localization motion
  for (auto iter = iter_1; iter != iter_2; iter++) {
    mat_motion_2d_image_ *= iter->motion;
    time_difference_ += iter->time_d;
  }
  // clean raw_motion_queue useless history
  while (raw_motion_queue_.begin() != iter_2) {
    raw_motion_queue_.pop_front();
  }
}

void PlaneMotion::update_motion_buffer(VehicleStatus vehicledata,
                                       double pre_image_timestamp,
                                       double image_timestamp) {
  MutexLock lock(&mutex_);
  for (int k = 0; k < static_cast<int>(mot_buffer_->size()); k++) {
    (*mot_buffer_)[k].motion *= mat_motion_2d_image_;
  }

  // set time_diff as image_time_diff
  time_difference_ = image_timestamp - pre_image_timestamp;
  vehicledata.time_d = time_difference_;
  // update motion
  vehicledata.motion = mat_motion_2d_image_;
  vehicledata.time_ts = image_timestamp;
  mot_buffer_->push_back(vehicledata);  // a new motion between images
  // reset motion buffer
  mat_motion_2d_image_ =
      Eigen::Matrix3f::Identity();  // reset image accumulated motion
  time_difference_ = 0;             // reset the accumulated time difference
}

bool PlaneMotion::find_motion_with_timestamp(double timestamp,
                                             VehicleStatus *vs) {
  MutexLock lock(&mutex_);
  ADEBUG << "mot_buffer_->size(): " << mot_buffer_->size();
  int i = static_cast<int>(mot_buffer_->size()) - 1;
  for (; i >= 0; i--) {
    if (std::abs(mot_buffer_->at(i).time_ts - timestamp) <
      std::numeric_limits<double>::epsilon()) {
      *vs = mot_buffer_->at(i);
      break;
    }
  }
  return (i >= 0);
}

void PlaneMotion::add_new_motion(VehicleStatus *vehicledata,
                                 double pre_image_timestamp,
                                 double image_timestamp,
                                 int motion_operation_flag) {
  while (!raw_motion_queue_.empty() &&
      vehicledata->time_ts < raw_motion_queue_.back().time_ts) {
      raw_motion_queue_.pop_back();
      ADEBUG << "pop ts : back ts" << std::to_string(vehicledata->time_ts)
             << " " << std::to_string(raw_motion_queue_.back().time_ts)
             << " " << raw_motion_queue_.size();
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
        accumulate_motion(pre_image_timestamp,
                          image_timestamp);
        update_motion_buffer(*vehicledata,
                            pre_image_timestamp,
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
    vehicledata->motion =  Eigen::Matrix3f::Identity();
    mot_buffer_->push_back(*vehicledata);
    ADEBUG << "pop and rest raw_buffer, mot_buffer: "
            << raw_motion_queue_.size();
  }
}
}  // namespace perception
}  // namespace apollo
