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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_CONVERT_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_CONVERT_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include "velodyne_pointcloud/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace velodyne {

// convert velodyne data to pointcloud and republish
class Convert {
 public:
  Convert() {}
  ~Convert();

  // init velodyne config struct from private_nh
  void init(ros::NodeHandle& node, ros::NodeHandle& private_nh);

 private:
  // convert velodyne data to pointcloudn and public
  void convert_packets_to_pointcloud(
      const velodyne_msgs::VelodyneScanUnified::ConstPtr& scan_msg);

  // RawData class for converting data to point cloud
  VelodyneParser* parser_;

  ros::Subscriber velodyne_scan_;
  ros::Publisher pointcloud_pub_;

  std::string topic_packets_;
  std::string topic_pointcloud_;

  /// configuration parameters, get config struct from velodyne_parser.h
  apollo::drivers::velodyne::Config config_;
  // queue size for ros node pub
  int queue_size_;
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_CONVERT_H_
