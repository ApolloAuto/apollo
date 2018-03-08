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

#include "modules/perception/obstacle/camera/motionmanager/vehicleplanemotion.h"

namespace apollo {
namespace perception {

PlaneMotion::PlaneMotion(int s) {
  init(s);
}

PlaneMotion::PlaneMotion(int s, bool sync_time_stamp, float time_unit)
    : time_unit_(time_unit) {
  init(s);
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

  float theta = time_unit_ * vehicledata->yaw_rate;
  float displacement = time_unit_ * vehicledata->velocity;

  Eigen::Rotation2Df rot2d(theta);
  Eigen::Vector2f trans;

  trans(0) = displacement * cos(theta);
  trans(1) = displacement * sin(theta);

  motion_2d.block(0, 0, 2, 2) = rot2d.toRotationMatrix().transpose();
  motion_2d.block(0, 2, 2, 1) = -rot2d.toRotationMatrix().transpose() * trans;

  vehicledata->motion = motion_2d;
}

void PlaneMotion::add_new_motion(VehicleStatus *vehicledata,
                                 float motion_time_dif, bool image_read) {
  generate_motion_matrix(vehicledata);  // compute vehicledata.motion

  // both CAN+IMU and image time stamp
  _mat_motion_2d_image =
      _mat_motion_2d_image * vehicledata->motion;  // accumulate CAN+IMU motion

  time_difference_ +=
      motion_time_dif;  // accumulate time diff before inserting into buffer

  if (image_read) {
    // image capture time stamp to insert the buffer for the accumulated motion
    for (int k = 0; k < static_cast<int>(mot_buffer_->size()); k++) {
      (*mot_buffer_)[k].motion *= _mat_motion_2d_image;
    }

    vehicledata->time_d = time_difference_;
    vehicledata->motion = _mat_motion_2d_image;
    mot_buffer_->push_back(*vehicledata);  // a new image frame is added
    _mat_motion_2d_image =
        Eigen::Matrix3f::Identity();  // reset image accumulated motion
    time_difference_ = 0;             // reset the accumulated time difference
  }
}

}  // namespace perception
}  // namespace apollo
