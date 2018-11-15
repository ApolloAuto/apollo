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

/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  Id
 */

/** @file
    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.
*/

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CONVERT_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CONVERT_H_

#include "modules/drivers/lidar_velodyne/pointcloud/rawdata.h"

#include "dynamic_reconfigure/server.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/drivers/lidar_velodyne/proto/cloud_node_conf.pb.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class Convert {
 public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Convert() {}

 private:
  void callback(const CloudNodeConf &config);
  void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<
      apollo::drivers::lidar_velodyne::CloudNodeConf> >
      srv_;

  boost::shared_ptr<RawData> data_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;

  /// configuration parameters
  typedef struct {
    int npackets;  ///< number of packets to combine
  } Config;
  Config config_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_CONVERT_H_
