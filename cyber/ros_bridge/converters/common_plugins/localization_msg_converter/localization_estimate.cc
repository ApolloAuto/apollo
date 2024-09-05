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

#include "cyber/ros_bridge/converters/common_plugins/localization_msg_converter/localization_estimate.h"  // NOLINT

namespace apollo {
namespace cyber {

bool LocalizationEstimate::ConvertMsg(
    InputTypes<InputMsgPtr>& in,
    OutputTypes<LocalizationMsgPtr, TransformMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto ros_odometry_ptr = std::get<0>(in.values);
  auto& ros_odometry = (*ros_odometry_ptr);
  auto localization_estimate = std::get<0>(out.values);
  auto tf = std::get<1>(out.values);

  auto unix_msg_time =
      ros_odometry.header.stamp.sec + ros_odometry.header.stamp.nanosec / 1e9;
  localization_estimate->mutable_header()->set_timestamp_sec(unix_msg_time);
  localization_estimate->mutable_header()->set_module_name("localization");
  localization_estimate->set_measurement_time(unix_msg_time);
  localization_estimate->mutable_pose()->mutable_position()->set_x(
      ros_odometry.pose.pose.position.x);
  localization_estimate->mutable_pose()->mutable_position()->set_y(
      ros_odometry.pose.pose.position.y);
  localization_estimate->mutable_pose()->mutable_position()->set_z(
      ros_odometry.pose.pose.position.z);
  localization_estimate->mutable_pose()->mutable_linear_velocity()->set_x(
      ros_odometry.twist.twist.linear.x);
  localization_estimate->mutable_pose()->mutable_linear_velocity()->set_y(
      ros_odometry.twist.twist.linear.y);
  localization_estimate->mutable_pose()->mutable_linear_velocity()->set_z(
      ros_odometry.twist.twist.linear.z);
  localization_estimate->mutable_pose()->mutable_angular_velocity()->set_x(
      ros_odometry.twist.twist.angular.x);
  localization_estimate->mutable_pose()->mutable_angular_velocity()->set_y(
      ros_odometry.twist.twist.angular.y);
  localization_estimate->mutable_pose()->mutable_angular_velocity()->set_z(
      ros_odometry.twist.twist.angular.z);
  localization_estimate->mutable_pose()->mutable_orientation()->set_qx(
      ros_odometry.pose.pose.orientation.x);
  localization_estimate->mutable_pose()->mutable_orientation()->set_qy(
      ros_odometry.pose.pose.orientation.y);
  localization_estimate->mutable_pose()->mutable_orientation()->set_qz(
      ros_odometry.pose.pose.orientation.z);
  localization_estimate->mutable_pose()->mutable_orientation()->set_qw(
      ros_odometry.pose.pose.orientation.w);

  auto single_tf = tf->add_transforms();
  single_tf->mutable_header()->set_timestamp_sec(unix_msg_time);
  single_tf->mutable_header()->set_frame_id("world");
  single_tf->set_child_frame_id("localization");
  single_tf->mutable_transform()->mutable_translation()->set_x(
      ros_odometry.pose.pose.position.x);
  single_tf->mutable_transform()->mutable_translation()->set_y(
      ros_odometry.pose.pose.position.y);
  single_tf->mutable_transform()->mutable_translation()->set_z(
      ros_odometry.pose.pose.position.z);
  single_tf->mutable_transform()->mutable_rotation()->set_qx(
      ros_odometry.pose.pose.orientation.x);
  single_tf->mutable_transform()->mutable_rotation()->set_qy(
      ros_odometry.pose.pose.orientation.y);
  single_tf->mutable_transform()->mutable_rotation()->set_qz(
      ros_odometry.pose.pose.orientation.z);
  single_tf->mutable_transform()->mutable_rotation()->set_qw(
      ros_odometry.pose.pose.orientation.w);
#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
