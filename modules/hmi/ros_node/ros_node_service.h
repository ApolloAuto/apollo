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

/**
 * @file ros_node_service.h
 * @brief the class of HMIRosNodeImpl
 */

#ifndef MODULES_HMI_ROS_NODE_ROS_NODE_SERVICE_H_
#define MODULES_HMI_ROS_NODE_ROS_NODE_SERVICE_H_

#include <mutex>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/hmi/proto/ros_node.grpc.pb.h"

/**
 * @namespace apollo::hmi
 * @brief apollo::hmi
 */
namespace apollo {
namespace hmi {

/**
 * @class HMIRosNodeImpl
 *
 * @brief Implementation of HMIRosNode service.
 */
class HMIRosNodeImpl final : public HMIRosNode::Service {
 public:
  /*
   * @brief Init the ROS node.
   */
  static void Init();

  /*
   * @brief Implementation of ChangeDrivingMode RPC.
   * @param context a pointer to the grpc context
   * @param point a pointer to an instance of ChangeDrivingModeRequest
   * @param feature a pointer to an instance of ChangeDrivingModeResponse
   * @return the grpc status
   */
  grpc::Status ChangeDrivingMode(grpc::ServerContext* context,
                                 const ChangeDrivingModeRequest* point,
                                 ChangeDrivingModeResponse* feature) override;

 private:
  // Monitor the driving mode by listening to Chassis message.
  static void MonitorDrivingMode(const apollo::canbus::Chassis& status);

  static std::mutex current_driving_mode_mutex_;
  static apollo::canbus::Chassis::DrivingMode current_driving_mode_;
};

}  // namespace hmi
}  // namespace apollo

#endif  // MODULES_HMI_ROS_NODE_ROS_NODE_SERVICE_H_
