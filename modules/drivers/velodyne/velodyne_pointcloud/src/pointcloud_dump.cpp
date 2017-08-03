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
  private_nh.param("save_folder", _save_folder, std::string(""));
  private_nh.param("topic_name", _topic_name, std::string(""));
  private_nh.param("file_prefix", _file_prefix, std::string(""));

  if (_save_folder == "" || _topic_name == "" || _file_prefix == "") {
    ROS_ERROR_STREAM("no file or topic name input");
  }

  _pointcloud_sub = node.subscribe(
      _topic_name, 1000, &PointCloudDump::save_callback, (PointCloudDump *)this,
      ros::TransportHints().tcpNoDelay(true));
}

void PointCloudDump::save_callback(const VPointCloud::ConstPtr &msg) {
  std::string ordered_file_path =
      _save_folder + "/" + _file_prefix +
      boost::lexical_cast<std::string>(msg->header.seq) + ".msg";
  dump_msg<VPointCloud>(*msg, ordered_file_path);
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
