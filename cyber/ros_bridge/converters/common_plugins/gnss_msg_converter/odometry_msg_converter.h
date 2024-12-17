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

#include "cyber/ros_bridge/converter_base/converter_interface.h"

#include "cyber/proto/simple.pb.h"
#include "modules/common_msgs/localization_msgs/gps.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"

#if __has_include("nav_msgs/msg/odometry.hpp")
#include "nav_msgs/msg/odometry.hpp"
#define ENABLE_ROS_MSG
#endif

#ifdef ENABLE_ROS_MSG
using RosOdometryMsg = nav_msgs::msg::Odometry;
#else
// fake wrap
using RosOdometryMsg = apollo::cyber::proto::SimpleMessage;
#endif

using OdometryOutputMsg = apollo::localization::Gps;

using RosOdometryMsgPtr = std::shared_ptr<RosOdometryMsg>;

using OdometryOutputMsgPtr = std::shared_ptr<OdometryOutputMsg>;

namespace apollo {
namespace cyber {

class OdometryMsgConverter
    : public apollo::cyber::RosApolloMessageConverter<
          InputTypes<RosOdometryMsgPtr>, OutputTypes<OdometryOutputMsgPtr>> {
 public:
  OdometryMsgConverter() {}
  ~OdometryMsgConverter() {}

  /**
   * @brief convert the message between ros and apollo
   *
   * @param InputTypes input message container
   * @param OutputTypes output message container
   * @return result, true for success
   */
  virtual bool ConvertMsg(InputTypes<RosOdometryMsgPtr>&,
                          OutputTypes<OdometryOutputMsgPtr>&);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::cyber::OdometryMsgConverter,
                                     apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
