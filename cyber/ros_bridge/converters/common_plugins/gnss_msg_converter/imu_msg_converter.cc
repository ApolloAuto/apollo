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

#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/imu_msg_converter.h"  // NOLINT
#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/quaternion_math.h"

namespace apollo {
namespace cyber {

bool ImuMsgConverter::ConvertMsg(
    InputTypes<RosImuMsgPtr>& in,
    OutputTypes<ImuMsgPtr, CorrectedImuMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto ros_imu_ptr = std::get<0>(in.values);
  auto& ros_imu = (*ros_imu_ptr);

  auto imu_msg = std::get<0>(out.values);
  auto corrected_imu_msg = std::get<1>(out.values);

  // ros header time is unix time,
  // while measurement_time of apollo imu is gps time
  double unix_msg_time =
      ros_imu.header.stamp.sec + ros_imu.header.stamp.nanosec / 1e9;
  imu_msg->mutable_header()->set_timestamp_sec(unix_msg_time);
  imu_msg->mutable_header()->set_module_name("gnss");

  imu_msg->set_measurement_time(unix_msg_time);
  imu_msg->set_measurement_span(0.0);
  imu_msg->mutable_linear_acceleration()->set_x(ros_imu.linear_acceleration.x);
  imu_msg->mutable_linear_acceleration()->set_y(ros_imu.linear_acceleration.y);
  imu_msg->mutable_linear_acceleration()->set_z(ros_imu.linear_acceleration.z);
  imu_msg->mutable_angular_velocity()->set_x(ros_imu.angular_velocity.x);
  imu_msg->mutable_angular_velocity()->set_y(ros_imu.angular_velocity.y);
  imu_msg->mutable_angular_velocity()->set_z(ros_imu.angular_velocity.z);

  corrected_imu_msg->mutable_header()->set_timestamp_sec(unix_msg_time);
  corrected_imu_msg->mutable_header()->set_module_name("gnss");
  corrected_imu_msg->mutable_imu()->mutable_linear_acceleration()->set_x(
      ros_imu.linear_acceleration.x);
  corrected_imu_msg->mutable_imu()->mutable_linear_acceleration()->set_y(
      ros_imu.linear_acceleration.y);
  corrected_imu_msg->mutable_imu()->mutable_linear_acceleration()->set_z(
      ros_imu.linear_acceleration.z);
  corrected_imu_msg->mutable_imu()->mutable_angular_velocity()->set_x(
      ros_imu.angular_velocity.x);
  corrected_imu_msg->mutable_imu()->mutable_angular_velocity()->set_y(
      ros_imu.angular_velocity.y);
  corrected_imu_msg->mutable_imu()->mutable_angular_velocity()->set_z(
      ros_imu.angular_velocity.z);
  double quaternion[4];
  double euler_angles[3];

  quaternion[0] = ros_imu.orientation.x;
  quaternion[1] = ros_imu.orientation.y;
  quaternion[2] = ros_imu.orientation.z;
  quaternion[3] = ros_imu.orientation.w;

  QuaternionToEuler(quaternion, euler_angles);
  auto heading = QuaternionToHeading(euler_angles[2]);
  corrected_imu_msg->mutable_imu()->mutable_euler_angles()->set_x(
      euler_angles[0]);
  corrected_imu_msg->mutable_imu()->mutable_euler_angles()->set_y(
      euler_angles[1]);
  corrected_imu_msg->mutable_imu()->mutable_euler_angles()->set_z(
      euler_angles[2]);

#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
