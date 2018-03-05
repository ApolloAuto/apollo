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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_MOTIONMANAGER_VEHICLEPLANEMOTION_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_MOTIONMANAGER_VEHICLEPLANEMOTION_H

#include <math.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <memory>

#include "modules/perception/obstacle/base/object_supplement.h"

namespace apollo {
namespace perception {

class PlaneMotion {
 public:
  explicit PlaneMotion(int s);
  explicit PlaneMotion(int s, bool sync_time_stamp, float time_unit);

  ~PlaneMotion(void);

 private:
  MotionBufferPtr _mot_buffer;
  int _buffer_size;
  float _time_unit;
  int _time_increment;     // the time increment units in motion input
  float _time_difference;  // the time difference for each buffer input
  Eigen::Matrix3f _mat_motion_2d_image;
  // motion matrix of accumulation through high sampling CAN+IMU input sequence
  void generate_motion_matrix(
      VehicleStatus *vehicledata);  // generate inverse motion

 public:
  void cleanbuffer() {
    if (_mot_buffer != nullptr) {
      _mot_buffer->clear();
      _mot_buffer = nullptr;
    }

    _mat_motion_2d_image = Eigen::Matrix3f::Identity();
  }

  void set_buffer_size(int s) {
    cleanbuffer();
    _buffer_size = s;
    // _mot_buffer.reserve(_buffer_size);
    if (_mot_buffer == nullptr) {
      _mot_buffer = std::make_shared<MotionBuffer>(_buffer_size);
    } else {
      _mot_buffer->set_capacity(_buffer_size);
    }
  }

  void init(int s) {
    set_buffer_size(s);
    _mat_motion_2d_image = Eigen::Matrix3f::Identity();
  }

  void add_new_motion(VehicleStatus *vehicledata, float motion_time_dif,
                      bool image_read);

  MotionBufferPtr get_buffer() {
    return _mot_buffer;
  }
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_MOTIONMANAGER_VEHICLEPLANEMOTION_H
