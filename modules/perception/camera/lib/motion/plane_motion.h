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

#pragma once

#include <cmath>
#include <cstdio>
#include <list>
#include <memory>
#include <mutex>

#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "modules/perception/base/object_supplement.h"

namespace apollo {
namespace perception {
namespace camera {

class PlaneMotion {
 public:
  explicit PlaneMotion(int s);

  ~PlaneMotion(void);
  enum { ACCUM_MOTION = 0, ACCUM_PUSH_MOTION, PUSH_ACCUM_MOTION, RESET };

 private:
  std::list<base::VehicleStatus> raw_motion_queue_;
  base::MotionBufferPtr mot_buffer_;
  std::mutex mutex_;
  int buffer_size_;
  int time_increment_;     // the time increment units in motion input
  float time_difference_;  // the time difference for each buffer input
  base::MotionType mat_motion_sensor_ = base::MotionType::Identity();
  // motion matrix of accumulation through high sampling CAN+IMU input sequence
  bool is_3d_motion_;
  void generate_motion_matrix(
      base::VehicleStatus *vehicledata);  // generate inverse motion
  void accumulate_motion(double start_time, double end_time);
  void update_motion_buffer(const base::VehicleStatus &vehicledata,
                            const double pre_image_timestamp,
                            const double image_timestamp);

 public:
  void cleanbuffer() {
    if (mot_buffer_ != nullptr) {
      mot_buffer_->clear();
      mot_buffer_ = nullptr;
    }

    mat_motion_sensor_ = base::MotionType::Identity();
  }

  void set_buffer_size(int s) {
    cleanbuffer();
    buffer_size_ = s;
    // mot_buffer_.reserve(buffer_size_);
    if (mot_buffer_ == nullptr) {
      mot_buffer_ = std::make_shared<base::MotionBuffer>(buffer_size_);
    } else {
      mot_buffer_->set_capacity(buffer_size_);
    }
  }

  void add_new_motion(double pre_image_timestamp, double image_timestamp,
                      int motion_operation_flag,
                      base::VehicleStatus *vehicledata);

  base::MotionBuffer get_buffer();
  bool find_motion_with_timestamp(double timestamp, base::VehicleStatus *vs);
  bool is_3d_motion() const { return is_3d_motion_; }
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
