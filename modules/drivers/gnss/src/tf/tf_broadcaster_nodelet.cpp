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

#include <geometry_msgs/TransformStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tf_broadcaster.h"

namespace apollo {
namespace drivers {
namespace gnss {

class TFBroadcasterNodelet : public nodelet::Nodelet {
 public:
  TFBroadcasterNodelet() {}
  ~TFBroadcasterNodelet() {}

 private:
  virtual void onInit();

  std::unique_ptr<TFBroadcaster> _tf_broadcaster;
};

void TFBroadcasterNodelet::onInit() {
  ros::NodeHandle& nh = getPrivateNodeHandle();

  _tf_broadcaster.reset(new TFBroadcaster(nh));
  _tf_broadcaster->init();
  ROS_INFO("Init tf broadcaster nodelet success.");
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelet_gnss.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(gnss_driver, TFBroadcasterNodelet,
                        apollo::drivers::gnss::TFBroadcasterNodelet,
                        nodelet::Nodelet);
