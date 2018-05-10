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

#include <ros/ros.h>

#include "driver.h"

int main(int argc, char** argv) {
  ROS_INFO("Velodyne driver node init");
  ros::init(argc, argv, "velodyne_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  apollo::drivers::velodyne::VelodyneDriver* dvr =
      apollo::drivers::velodyne::VelodyneDriverFactory::create_driver(
          private_nh);
  if (dvr == nullptr) {
    ROS_BREAK();
  }
  dvr->init(node);

  // loop until shut down or end of file
  while (ros::ok() && dvr->poll()) {
    ros::spinOnce();
  }

  return 0;
}
