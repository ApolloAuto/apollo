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

#include "velodyne_pointcloud/convert.h"

#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ROS_INFO("Point cloud node init");
  ros::init(argc, argv, "cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // create conversion class, which subscribes to raw data
  apollo::drivers::velodyne::Convert convert;
  convert.init(node, private_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
