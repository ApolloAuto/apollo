/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_DRIVERS_ROBOSENSE_RSLIDAR_POINTCLOUD_CONVERT_H_
#define MODULES_DRIVERS_ROBOSENSE_RSLIDAR_POINTCLOUD_CONVERT_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include "rslidar_pointcloud/rslidarParser.h"
#include "rslidar_msgs/rslidarScan.h"

namespace apollo {
namespace drivers {
namespace rslidar {

// convert rslidar data to pointcloud and republish
class Convert {
 public:
 Convert() {}
  ~Convert();

  // init rslidar config struct from private_nh
  void init(ros::NodeHandle& node, ros::NodeHandle& private_nh);

 private:
  // convert rslidar data to pointcloudn and public
  void convert_packets_to_pointcloud(
      const rslidar_msgs::rslidarScan::ConstPtr& scan_msg);

  // RawData class for converting data to point cloud
  rslidarParser* data_;
  
  ros::Subscriber rslidar_scan_;
  ros::Publisher pointcloud_pub_;

  std::string topic_packets_;
  std::string topic_pointcloud_;
 
  int queue_size_;
};

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo

#endif  
