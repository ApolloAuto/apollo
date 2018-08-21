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

/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  Id
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef MODULES_DRIVERS_VELODYN_DRIVER_DRIVER_H_
#define MODULES_DRIVERS_VELODYN_DRIVER_DRIVER_H_

#include <string>

#include "dynamic_reconfigure/server.h"
#include "ros/ros.h"

#include "modules/drivers/lidar_velodyne/driver/input.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class VelodyneDriver {
 public:
  VelodyneDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(void);

 private:
  /// Callback for dynamic reconfigure
  // void callback(velodyne_driver::VelodyneNodeConfig &config, uint32_t level);

  /// Pointer to dynamic reconfigure service srv_
  // boost::shared_ptr<
  //     dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig> >
  //     srv_;

  // configuration parameters
  struct {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;  ///< time in seconds added to each velodyne time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  // diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  // boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYN_DRIVER_DRIVER_H__
