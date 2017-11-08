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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_pointcloud_dump_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_pointcloud_dump_H_

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>

#include "velodyne_pointcloud/velodyne_parser.h"

namespace apollo {
namespace drivers {
namespace velodyne {

// save msg to file: file name is file_prefix_ + msg.seq + .msg
class PointCloudDump {
 public:
  PointCloudDump(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PointCloudDump() {}

 private:
  void save_callback(const VPointCloud::ConstPtr &msg);

  // save msg folder
  std::string save_folder_;
  // sub topic name
  std::string topic_name_;
  // save file prefix,file will be prefix_msgseq.msg
  std::string file_prefix_;
  ros::Subscriber pointcloud_sub_;
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_pointcloud_dump_H_
