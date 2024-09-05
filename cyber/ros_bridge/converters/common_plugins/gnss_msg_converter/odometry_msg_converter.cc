/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/odometry_msg_converter.h"  // NOLINT

namespace apollo {
namespace cyber {

bool OdometryMsgConverter::ConvertMsg(InputTypes<RosOdometryMsgPtr>& in,
                                      OutputTypes<OdometryOutputMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto ros_odometry_ptr = std::get<0>(in.values);
  auto& ros_odometry = (*ros_odometry_ptr);

  auto odometry_msg = std::get<0>(out.values);

  auto unix_msg_time =
      ros_odometry.header.stamp.sec + ros_odometry.header.stamp.nanosec / 1e9;
  odometry_msg->mutable_header()->set_timestamp_sec(unix_msg_time);
  odometry_msg->mutable_header()->set_module_name("gnss");
  odometry_msg->mutable_localization()->mutable_position()->set_x(
      ros_odometry.pose.pose.position.x);
  odometry_msg->mutable_localization()->mutable_position()->set_y(
      ros_odometry.pose.pose.position.y);
  odometry_msg->mutable_localization()->mutable_position()->set_z(
      ros_odometry.pose.pose.position.z);
  odometry_msg->mutable_localization()->mutable_linear_velocity()->set_x(
      ros_odometry.twist.twist.linear.x);
  odometry_msg->mutable_localization()->mutable_linear_velocity()->set_y(
      ros_odometry.twist.twist.linear.y);
  odometry_msg->mutable_localization()->mutable_linear_velocity()->set_z(
      ros_odometry.twist.twist.linear.z);
  odometry_msg->mutable_localization()->mutable_angular_velocity()->set_x(
      ros_odometry.twist.twist.angular.x);
  odometry_msg->mutable_localization()->mutable_angular_velocity()->set_y(
      ros_odometry.twist.twist.angular.y);
  odometry_msg->mutable_localization()->mutable_angular_velocity()->set_z(
      ros_odometry.twist.twist.angular.z);
  odometry_msg->mutable_localization()->mutable_orientation()->set_qx(
      ros_odometry.pose.pose.orientation.x);
  odometry_msg->mutable_localization()->mutable_orientation()->set_qy(
      ros_odometry.pose.pose.orientation.y);
  odometry_msg->mutable_localization()->mutable_orientation()->set_qz(
      ros_odometry.pose.pose.orientation.z);
  odometry_msg->mutable_localization()->mutable_orientation()->set_qw(
      ros_odometry.pose.pose.orientation.w);

#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
