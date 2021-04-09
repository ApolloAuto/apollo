/******************************************************************************
 * Modification Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 * Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 * License: Modified BSD Software License Agreement
 *
 * Id
 **/

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include "modules/drivers/lidar_velodyne/driver/driver_nodelet.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

void DriverNodelet::OnInit() {
  // start the driver
  dvr_.reset(new VelodyneDriver(nh_, private_nh_));

  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&DriverNodelet::DevicePoll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::DevicePoll() {
  while (ros::ok()) {
    // poll device until end of file
    running_ = dvr_->poll();
    if (!running_) break;
  }
  running_ = false;
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
