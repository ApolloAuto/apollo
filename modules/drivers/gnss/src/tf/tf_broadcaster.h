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

#ifndef MODULES_DRIVERS_GNSS_TF_BROADCASTER_H_
#define MODULES_DRIVERS_GNSS_TF_BROADCASTER_H_

#include <geometry_msgs/TransformStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include "gnss/parser.h"

#include "modules/localization/proto/gps.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

class TFBroadcaster {
 public:
  TFBroadcaster(const ros::NodeHandle& nh) : _nh(nh) {}
  ~TFBroadcaster() {}

  void init();

 private:
  std::string _odometry_topic;
  std::string _frame_id;
  std::string _child_frame_id;
  ros::NodeHandle _nh;
  ros::Subscriber _odometry_sub;
  tf2_ros::TransformBroadcaster _broadcaster;

  void gps_to_transform_stamped(const localization::Gps& gps,
                                geometry_msgs::TransformStamped* transform);
  void odometry_callback(const boost::shared_ptr<const localization::Gps>& gps);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_TF_BROADCASTER_H_
