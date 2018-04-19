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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_MOTION_PLANE_MOTION_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_MOTION_PLANE_MOTION_H_

#include <cmath>
#include <cstdio>
#include <list>
#include <memory>

#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/obstacle/base/object_supplement.h"

namespace apollo {
namespace perception {

class PlaneMotion {
 public:
  explicit PlaneMotion(int s);

  ~PlaneMotion(void);
  enum { ACCUM_MOTION = 0, ACCUM_PUSH_MOTION, PUSH_ACCUM_MOTION, RESET};

 private:
  std::list<VehicleStatus> raw_motion_queue_;
  MotionBufferPtr mot_buffer_;
  Mutex mutex_;
  int buffer_size_;
  int time_increment_;     // the time increment units in motion input
  float time_difference_;  // the time difference for each buffer input
  Eigen::Matrix3f mat_motion_2d_image_ = Eigen::Matrix3f::Identity();
  // motion matrix of accumulation through high sampling CAN+IMU input sequence
  void generate_motion_matrix(
      VehicleStatus *vehicledata);  // generate inverse motion
  void accumulate_motion(double start_time, double end_time);
  void update_motion_buffer(VehicleStatus vehicledata,
        double pre_image_timestamp, double image_timestamp);

 public:
  void cleanbuffer() {
    if (mot_buffer_ != nullptr) {
      mot_buffer_->clear();
      mot_buffer_ = nullptr;
    }

    mat_motion_2d_image_ = Eigen::Matrix3f::Identity();
  }

  void set_buffer_size(int s) {
    cleanbuffer();
    buffer_size_ = s;
    // mot_buffer_.reserve(buffer_size_);
    if (mot_buffer_ == nullptr) {
      mot_buffer_ = std::make_shared<MotionBuffer>(buffer_size_);
    } else {
      mot_buffer_->set_capacity(buffer_size_);
    }
  }

  // void init(int s) { set_buffer_size(s); }

//   void add_new_motion(VehicleStatus *vehicledata, float motion_time_dif,
//                      int motion_operation_flag);

  void add_new_motion(VehicleStatus *vehicledata,
                      double pre_image_timestamp,
                      double image_timestamp,
                      int motion_operation_flag);

  MotionBufferPtr get_buffer() { return mot_buffer_; }
  bool find_motion_with_timestamp(double timestamp, VehicleStatus *vs);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_MOTION_PLANE_MOTION_H_
