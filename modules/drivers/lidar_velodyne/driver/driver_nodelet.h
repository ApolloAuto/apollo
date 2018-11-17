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

#ifndef MODULES_DRIVERS_LIDAR_VELODYN_DRIVER_DRIVER_NODELET_H_
#define MODULES_DRIVERS_LIDAR_VELODYN_DRIVER_DRIVER_NODELET_H_

#include <string>

#include "boost/thread.hpp"
#include "ros/ros.h"

#include "modules/common/log.h"
#include "modules/drivers/lidar_velodyne/driver/driver.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class DriverNodelet {
 public:
  DriverNodelet() : running_(false) {}
  ~DriverNodelet() {
    if (running_) {
      AINFO << "shutting down driver thread";
      running_ = false;
      deviceThread_->join();
      AINFO << "driver thread stopped";
    }
  }

  virtual void OnInit();

 private:
  virtual void DevicePoll();

  volatile bool running_;  ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<VelodyneDriver> dvr_;  ///< driver implementation class

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYN_DRIVER_DRIVER_NODELET_H_
