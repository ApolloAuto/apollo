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

#ifndef CYBER_APOLLO_ROS_BASE_H_
#define CYBER_APOLLO_ROS_BASE_H_

#include <memory>
#include <string>
#include <tuple>

#include "cyber/cyber.h"
#include "cyber/ros_bridge/converter_base/message_converter.h"

namespace apollo {
namespace cyber {

template <typename... Types>
struct InputTypes {
  std::tuple<Types...> values;
  static constexpr size_t NInputs = sizeof...(Types);
};

template <typename... Types>
struct OutputTypes {
  std::tuple<Types...> values;
  static constexpr size_t NOutputs = sizeof...(Types);
};

template <typename InputTypes, typename OutputTypes>
class ApolloRosMessageConverter : public MessageConverter {
 public:
  ApolloRosMessageConverter() {}
  ~ApolloRosMessageConverter() override {}

  bool Init() override {
    AERROR << "input output not support";
    return false;
  };

 protected:
  virtual bool ConvertMsg(InputTypes& input, OutputTypes& output) = 0;
};

template <typename InputTypes, typename OutputTypes>
class RosApolloMessageConverter : public MessageConverter {
 public:
  RosApolloMessageConverter() {}
  ~RosApolloMessageConverter() override {}

  bool Init() override {
    AERROR << "input output not support";
    return false;
  };

 protected:
  virtual bool ConvertMsg(InputTypes& input, OutputTypes& output) = 0;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_APOLLO_ROS_MESSAGE_CONVERTER_H_
