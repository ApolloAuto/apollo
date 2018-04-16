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

#include "pandora_pointcloud/compensator.h"

#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ROS_INFO("Point cloud node init");
  ros::init(argc, argv, "pandora_compensator_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  apollo::drivers::pandora::Compensator compensator(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
