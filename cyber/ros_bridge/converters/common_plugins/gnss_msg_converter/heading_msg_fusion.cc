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

#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/heading_msg_fusion.h"  // NOLINT
#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/quaternion_math.h"

namespace apollo {
namespace cyber {

bool HeadingMsgFusion::ConvertMsg(
    InputTypes<RosNavMsgPtr, RosOdometryMsgPtr>& in,
    OutputTypes<OutputMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto ros_nav_ptr = std::get<0>(in.values);
  auto ros_odometry_ptr = std::get<1>(in.values);
  auto& ros_nav = (*ros_nav_ptr);
  auto& ros_odometry = (*ros_odometry_ptr);

  auto heading_msg = std::get<0>(out.values);

  auto unix_msg_time =
      ros_nav.header.stamp.sec + ros_nav.header.stamp.nanosec / 1e9;

  double quaternion[4];
  double euler_angles[3];

  quaternion[0] = ros_odometry.pose.pose.orientation.x;
  quaternion[1] = ros_odometry.pose.pose.orientation.y;
  quaternion[2] = ros_odometry.pose.pose.orientation.z;
  quaternion[3] = ros_odometry.pose.pose.orientation.w;

  QuaternionToEuler(quaternion, euler_angles);
  auto pitch = euler_angles[1];
  auto heading = QuaternionToHeading(euler_angles[2]);

  heading_msg->mutable_header()->set_timestamp_sec(unix_msg_time);
  heading_msg->mutable_header()->set_module_name("gnss");
  heading_msg->set_measurement_time(unix_msg_time);

  if (ros_nav.status.status == -1) {
    heading_msg->set_solution_status(19);
    heading_msg->set_position_type(0);
  } else if (ros_nav.status.status == 0) {
    heading_msg->set_solution_status(0);
    heading_msg->set_position_type(16);
  } else if (ros_nav.status.status == 1) {
    heading_msg->set_solution_status(0);
    heading_msg->set_position_type(18);
  } else if (ros_nav.status.status == 2) {
    heading_msg->set_solution_status(0);
    heading_msg->set_position_type(50);
  }
  heading_msg->set_heading(heading);
  heading_msg->set_pitch(pitch);

  /*
    pose covariance:
    [
      xx,     xy,     xz,     xroll,      xpitch,     xyaw,
      yx,     yy,     yz,     yroll,      ypitch,     yyaw,
      zx,     zy,     zz,     zroll,      zpitch,     zzaw,
      rollx,  rolly,  rollz,  rollroll,   rollpitch,  rollyaw,
      pitchx, pitchy, pitchz, pitchroll,  pitchpitch, pitchyaw,
      yawx,   yawy,   yawz,   yawroll,    yawpitch,   yawyaw
    ]
    thus pitch_stddev = sqrt(covariance[28]), yaw_stddev = sqrt(covariance[35])
  */

  auto covariance = ros_odometry.pose.covariance;
  auto pitch_variance = covariance[28];
  auto yaw_variance = covariance[35];

  double pitch_stddev = sqrt(pitch_variance);
  double yaw_stddev = sqrt(yaw_variance);

  heading_msg->set_heading_std_dev(yaw_stddev);
  heading_msg->set_pitch_std_dev(pitch_stddev);

#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
