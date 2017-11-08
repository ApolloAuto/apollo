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

#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

#include "velodyne_pointcloud/pointcloud_dump.h"
#include "velodyne_pointcloud/util.h"

namespace apollo {
namespace drivers {
namespace velodyne {

PointCloudDump::PointCloudDump(ros::NodeHandle node,
                               ros::NodeHandle private_nh) {
  private_nh.param("save_folder", save_folder_, std::string(""));
  private_nh.param("topic_name", topic_name_, std::string(""));
  private_nh.param("file_prefix", file_prefix_, std::string(""));

  if (save_folder_ == "" || topic_name_ == "" || file_prefix_ == "") {
    ROS_ERROR_STREAM("no file or topic name input");
  }

  pointcloud_sub_ = node.subscribe(
      topic_name_, 1000, &PointCloudDump::save_callback, (PointCloudDump *)this,
      ros::TransportHints().tcpNoDelay(true));
}

void PointCloudDump::save_callback(const VPointCloud::ConstPtr &msg) {
  std::string ordered_file_path =
      save_folder_ + "/" + file_prefix_ +
      boost::lexical_cast<std::string>(msg->header.seq) + ".msg";
  dump_msg<VPointCloud>(*msg, ordered_file_path);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
