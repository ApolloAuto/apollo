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

#pragma once

#include <algorithm>
#include <atomic>
#include <cstring>
#include <memory>
#include <string>

#include "cyber/proto/simple.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/converter_base/converter_interface.h"

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj_api.h>  // NOLINT

#include <Eigen/Dense>  // NOLINT

#if __has_include("sensor_msgs/msg/nav_sat_fix.hpp")
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#define ROS_NAVFOUND_FOUND
#endif

#if __has_include("sensor_msgs/msg/imu.hpp")
#include "sensor_msgs/msg/imu.hpp"
#define ROS_IMU_FOUND
#endif

#if __has_include("nav_msgs/msg/odometry.hpp")
#include "nav_msgs/msg/odometry.hpp"
#define ROS_ODOMETRY_FOUND
#endif

#if defined(ROS_NAVFOUND_FOUND) && defined(ROS_IMU_FOUND) && \
    defined(ROS_ODOMETRY_FOUND)
#define ENABLE_ROS_MSG
#endif

#ifdef ENABLE_ROS_MSG
using RosNavMsg = sensor_msgs::msg::NavSatFix;
using RosImuMsg = sensor_msgs::msg::Imu;
using RosWrapMsg = std_msgs::msg::Header;
using RosOdomMsg = nav_msgs::msg::Odometry;
#else
// fake wrap
using RosNavMsg = apollo::cyber::proto::SimpleMessage;
using RosImuMsg = apollo::cyber::proto::SimpleMessage;
using RosWrapMsg = apollo::cyber::proto::SimpleMessage;
#endif

using OutputWrapMsg = apollo::cyber::proto::SimpleMessage;

using RosWrapMsgPtr = std::shared_ptr<RosWrapMsg>;
using OutputWrapMsgPtr = std::shared_ptr<OutputWrapMsg>;

namespace apollo {
namespace cyber {

class OdometryParser
    : public apollo::cyber::RosApolloMessageConverter<
          InputTypes<RosWrapMsgPtr>, OutputTypes<OutputWrapMsgPtr>> {
 public:
  OdometryParser() {
#ifdef ENABLE_ROS_MSG
    imu_sub_ = ros_node_->create_subscription<RosImuMsg>(
        converter_conf_.ros_topic_name_0(), 10,
        std::bind(&OdometryParser::imuCallback, this, std::placeholders::_1));
    gps_sub_ = ros_node_->create_subscription<RosNavMsg>(
        converter_conf_.ros_topic_name_1(), 10,
        std::bind(&OdometryParser::gpsCallback, this, std::placeholders::_1));
    odom_pub_ = ros_node_->create_publisher<RosOdomMsg>(
        converter_conf_.ros_topic_name_2(), 10);
    timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OdometryParser::publishOdometry, this));
#endif

    wgs84pj_source_ = pj_init_plus("+proj=latlong +ellps=WGS84");
    X_ = Eigen::VectorXd::Zero(9);
    P_ = Eigen::MatrixXd::Identity(9, 9);
    F_ = Eigen::MatrixXd::Identity(9, 9);
    H_ = Eigen::MatrixXd::Identity(9, 9);
    R_ = Eigen::MatrixXd::Identity(9, 9) * 0.1;
    Q_ = Eigen::MatrixXd::Identity(9, 9) * 0.01;
    last_time_ = apollo::cyber::Time::Now().ToSecond();
  }
  ~OdometryParser() {}

  void imuCallback(const RosImuMsg& msg);

  void gpsCallback(const RosNavMsg& msg);

  void publishOdometry();

  /**
   * @brief convert the message between ros and apollo
   *
   * @param InputTypes input message container
   * @param OutputTypes output message container
   * @return result, true for success
   */
  virtual bool ConvertMsg(InputTypes<RosWrapMsgPtr>&,
                          OutputTypes<OutputWrapMsgPtr>&);

 private:
  projPJ wgs84pj_source_ = NULL;
  projPJ utm_target_ = NULL;
#ifdef ENABLE_ROS_MSG
  ::rclcpp::SubscriptionBase::SharedPtr imu_sub_;
  ::rclcpp::SubscriptionBase::SharedPtr gps_sub_;
  ::rclcpp::Publisher<RosOdomMsg>::SharedPtr odom_pub_;
  ::rclcpp::TimerBase::SharedPtr timer_;
#endif

  std::atomic<bool> imu_received_ = false;
  std::atomic<bool> gps_received_ = false;

  double last_time_;

  double quaternion_[4];

  // Kalman filter
  Eigen::VectorXd X_;  // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
  Eigen::MatrixXd P_;  // Covariance matrix
  Eigen::MatrixXd F_;  // State transition matrix
  Eigen::MatrixXd H_;  // Measurement matrix
  Eigen::MatrixXd R_;  // Measurement noise covariance matrix
  Eigen::MatrixXd Q_;  // Process noise covariance matrix
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::cyber::OdometryParser,
                                     apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
