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

#include <ros/ros.h>

#include "driver.h"

#include "std_msgs/String.h"

//using namespace rs_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char** argv) {
  ROS_INFO("Rslidar driver node init");
  ros::init(argc, argv, "rsdriver");		//add driver
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  apollo::drivers::rslidar::RslidarDriver* dvr =
      apollo::drivers::rslidar::RslidarDriverFactory::create_driver(
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
