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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_LASER_SCAN_LASER_SCAN_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_LASER_SCAN_LASER_SCAN_H_

#include "boost/thread/lock_guard.hpp"
#include "boost/thread/mutex.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class VelodyneLaserScan {
 public:
  VelodyneLaserScan(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);  // NOLINT

 private:
  boost::mutex connect_mutex_;
  void connectCb();
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  struct {
    int ring = 0;
    double resolution = 0.0;
  } cfg_;

  // VelodyneLaserScanConfig cfg_;
  // dynamic_reconfigure::Server<VelodyneLaserScanConfig> srv_;
  // void reconfig(VelodyneLaserScanConfig& config, uint32_t level);

  unsigned int ring_count_ = 0;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_LASER_SCAN_LASER_SCAN_H_
