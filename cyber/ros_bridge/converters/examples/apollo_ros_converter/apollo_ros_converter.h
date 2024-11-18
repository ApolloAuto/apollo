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

#include <memory>

#include "cyber/ros_bridge/converter_base/converter_interface.h"

#include "cyber/proto/simple.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"

#if __has_include("std_msgs/msg/string.hpp")
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#define ENABLE_ROS_MSG
#endif

using InputMsg0 = apollo::cyber::proto::SimpleMessage;
#ifdef ENABLE_ROS_MSG
using OutputMsg0 = std_msgs::msg::String;
#else
using OutputMsg0 = apollo::cyber::proto::SimpleRepeatedMessage;
#endif
using OutputMsg0Ptr = std::shared_ptr<OutputMsg0>;
using InputMsg0Ptr = std::shared_ptr<InputMsg0>;

namespace apollo {
namespace cyber {

class ApolloRosConverter
    : public apollo::cyber::ApolloRosMessageConverter<
          InputTypes<InputMsg0Ptr>, OutputTypes<OutputMsg0Ptr>> {
 public:
  ApolloRosConverter() {}
  ~ApolloRosConverter() {}

  /**
   * @brief convert the message between ros and apollo
   *
   * @param InputMsgPtr shared pointer of input message
   * @param OutputMsgPtr shared pointer of output message
   * @return result, true for success
   */
  virtual bool ConvertMsg(InputTypes<InputMsg0Ptr>&,
                          OutputTypes<OutputMsg0Ptr>&);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::cyber::ApolloRosConverter,
                                     apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
