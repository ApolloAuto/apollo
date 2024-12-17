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

#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/odometry_parser.h"  // NOLINT
#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/quaternion_math.h"

namespace apollo {
namespace cyber {

void OdometryParser::imuCallback(std::shared_ptr<RosImuMsg> msg) {
#ifdef ENABLE_ROS_MSG
  imu_received_.store(true);
  auto current_time = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  double dt = current_time - last_time_;
  last_time_ = current_time;

  F_(0, 3) = dt;
  F_(1, 4) = dt;
  F_(2, 5) = dt;

  X_ = F_ * X_;
  P_ = F_ * P_ * F_.transpose() + Q_;

  X_(3) += msg->linear_acceleration.x * dt;
  X_(4) += msg->linear_acceleration.y * dt;
  X_(5) += msg->linear_acceleration.z * dt;

  double euler_angles[3];

  quaternion_[0] = msg->orientation.x;
  quaternion_[1] = msg->orientation.y;
  quaternion_[2] = msg->orientation.z;
  quaternion_[3] = msg->orientation.w;

  QuaternionToEuler(quaternion_, euler_angles);
  auto roll = euler_angles[0];
  auto pitch = euler_angles[1];
  auto yaw = euler_angles[2];

  X_(6) = roll;
  X_(7) = pitch;
  X_(8) = yaw;
#endif
}

void OdometryParser::gpsCallback(std::shared_ptr<RosNavMsg> msg) {
#ifdef ENABLE_ROS_MSG
  gps_received_.store(true);
  constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
  Eigen::VectorXd Z(9);

  double lon = msg->longitude;
  double lat = msg->latitude;
  double px = lon * DEG_TO_RAD_LOCAL;
  double py = lat * DEG_TO_RAD_LOCAL;

  if (utm_target_ == NULL) {
    std::string proj4_text;
    int zone = static_cast<int>(std::ceil((lon + 180.0) / 6.0));
    proj4_text = "+proj=utm +zone=" + std::to_string(zone) +
                 " +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    utm_target_ = pj_init_plus(proj4_text.c_str());
  }
  pj_transform(wgs84pj_source_, utm_target_, 1, 1, &px, &py, NULL);

  Z << px, py, msg->altitude, 0, 0, 0, 0, 0, 0;

  Eigen::MatrixXd y = Z - H_ * X_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  X_ = X_ + K * y;
  P_ = (Eigen::MatrixXd::Identity(9, 9) - K * H_) * P_;
#endif
}

void OdometryParser::publishOdometry() {
#ifdef ENABLE_ROS_MSG
  if (!gps_received_.load() || !imu_received_.load()) {
    return;
  }
  RosOdomMsg odom_msg;
  odom_msg.header.stamp = ros_node_->get_clock()->now();

  odom_msg.pose.pose.position.x = X_(0);
  odom_msg.pose.pose.position.y = X_(1);
  odom_msg.pose.pose.position.z = X_(2);

  odom_msg.pose.pose.orientation.x = quaternion_[0];
  odom_msg.pose.pose.orientation.y = quaternion_[1];
  odom_msg.pose.pose.orientation.z = quaternion_[2];
  odom_msg.pose.pose.orientation.w = quaternion_[3];

  odom_msg.twist.twist.linear.x = X_(3);
  odom_msg.twist.twist.linear.y = X_(4);
  odom_msg.twist.twist.linear.z = X_(5);

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i < 3 && j < 3) {
        odom_msg.pose.covariance[i * 6 + j] = P_(i, j);
      } else if (i >= 3 && j >= 3) {
        odom_msg.pose.covariance[i * 6 + j] = P_(i + 3, j + 3);
      }
    }
  }

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom_msg.twist.covariance[i * 6 + j] = P_(i + 3, j + 3);
    }
  }

  odom_pub_->publish(odom_msg);
  imu_received_.store(false);
  imu_received_.store(false);
#endif
}

bool OdometryParser::ConvertMsg(InputTypes<RosWrapMsgPtr>& in,
                                OutputTypes<OutputWrapMsgPtr>& out) {
  // do nothing
  return true;
}

}  // namespace cyber
}  // namespace apollo

