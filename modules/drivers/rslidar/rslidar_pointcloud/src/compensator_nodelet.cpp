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

//#include "rslidar_pointcloud/compensator.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace apollo {
namespace drivers {
namespace rslidar {

class CompensatorNodelet : public nodelet::Nodelet {
 public:
  CompensatorNodelet() {}
  ~CompensatorNodelet() {}

 private:
  virtual void onInit();
};

/** @brief Nodelet initialization. */
void CompensatorNodelet::onInit() {

  ROS_INFO("Compensator nodelet init");
}

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo

// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(rslidar_pointcloud, CompensatorNodelet,
                        apollo::drivers::rslidar::CompensatorNodelet,
                        nodelet::Nodelet);
