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
#include <cstring>
#include <memory>
#include <string>

#include "cyber/proto/simple.pb.h"
#include "modules/common_msgs/localization_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/converter_base/converter_interface.h"

#if __has_include("sensor_msgs/msg/imu.hpp")
#include "sensor_msgs/msg/imu.hpp"
#define ENABLE_ROS_MSG
#endif

#ifdef ENABLE_ROS_MSG
using RosImuMsg = sensor_msgs::msg::Imu;
#else
// fake wrap
using RosImuMsg = apollo::cyber::proto::SimpleMessage;
#endif

using ImuMsg = apollo::drivers::gnss::Imu;
using CorrectedImuMsg = apollo::localization::CorrectedImu;

using RosImuMsgPtr = std::shared_ptr<RosImuMsg>;

using ImuMsgPtr = std::shared_ptr<apollo::drivers::gnss::Imu>;
using CorrectedImuMsgPtr = std::shared_ptr<apollo::localization::CorrectedImu>;

namespace apollo {
namespace cyber {

class ImuMsgConverter : public apollo::cyber::RosApolloMessageConverter<
                            InputTypes<RosImuMsgPtr>,
                            OutputTypes<ImuMsgPtr, CorrectedImuMsgPtr>> {
 public:
  ImuMsgConverter() {}
  ~ImuMsgConverter() {}

  /**
   * @brief convert the message between ros and apollo
   *
   * @param InputTypes input message container
   * @param OutputTypes output message container
   * @return result, true for success
   */
  virtual bool ConvertMsg(InputTypes<RosImuMsgPtr>&,
                          OutputTypes<ImuMsgPtr, CorrectedImuMsgPtr>&);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::cyber::ImuMsgConverter,
                                     apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
