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
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/converter_base/converter_interface.h"

#if __has_include("sensor_msgs/msg/nav_sat_fix.hpp")
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#define ROS_NAVFOUND_FOUND
#endif

#if __has_include("nav_msgs/msg/odometry.hpp")
#include "nav_msgs/msg/odometry.hpp"
#define ROS_ODOMETRY_FOUND
#endif

#if defined(ROS_NAVFOUND_FOUND) && defined(ROS_ODOMETRY_FOUND)
#define ENABLE_ROS_MSG
#endif

#ifdef ENABLE_ROS_MSG
using RosNavMsg = sensor_msgs::msg::NavSatFix;
using RosOdometryMsg = nav_msgs::msg::Odometry;
#else
// fake wrap
using RosNavMsg = apollo::cyber::proto::SimpleMessage;
using RosOdometryMsg = apollo::cyber::proto::SimpleMessage;
#endif

using OutputMsg = apollo::drivers::gnss::Heading;

using RosNavMsgPtr = std::shared_ptr<RosNavMsg>;
using RosOdometryMsgPtr = std::shared_ptr<RosOdometryMsg>;

using OutputMsgPtr = std::shared_ptr<OutputMsg>;

namespace apollo {
namespace cyber {

class HeadingMsgFusion : public apollo::cyber::RosApolloMessageConverter<
                             InputTypes<RosNavMsgPtr, RosOdometryMsgPtr>,
                             OutputTypes<OutputMsgPtr>> {
 public:
  HeadingMsgFusion() {}
  ~HeadingMsgFusion() {}

  /**
   * @brief convert the message between ros and apollo
   *
   * @param InputTypes input message container
   * @param OutputTypes output message container
   * @return result, true for success
   */
  virtual bool ConvertMsg(InputTypes<RosNavMsgPtr, RosOdometryMsgPtr>&,
                          OutputTypes<OutputMsgPtr>&);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::cyber::HeadingMsgFusion,
                                     apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
