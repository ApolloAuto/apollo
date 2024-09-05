/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_ROS_APOLLO_BASE_H_
#define CYBER_ROS_APOLLO_BASE_H_

#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/ros_bridge/converter_base/message_converter.h"

namespace apollo {
namespace cyber {

template <typename R0 = NullType, typename R1 = NullType,
          typename R2 = NullType, typename R3 = NullType,
          typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType, typename M3 = NullType>
class RosApolloMessageConverter : public MessageConverter {
 public:
  RosApolloMessageConverter() {}
  ~RosApolloMessageConverter() override {}

  bool Init() override;

 protected:
  virtual bool ConvertMsg(
      const std::shared_ptr<R0>& ros_msg0, const std::shared_ptr<R1>& ros_msg1,
      const std::shared_ptr<R2>& ros_msg2, const std::shared_ptr<R3>& ros_msg3,
      std::shared_ptr<M0>& apollo_msg0, std::shared_ptr<M1>& apollo_msg1,
      std::shared_ptr<M2>& apollo_msg2, std::shared_ptr<M3>& apollo_msg3) = 0;

#ifdef RCLCPP__RCLCPP_HPP_
 private:
  void TopicCallback(const std::shared_ptr<R0>& ros_msg0,
                     const std::shared_ptr<R1>& ros_msg1,
                     const std::shared_ptr<R2>& ros_msg2,
                     const std::shared_ptr<R3>& ros_msg3);
#endif
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_APOLLO_ROS_MESSAGE_CONVERTER_H_
