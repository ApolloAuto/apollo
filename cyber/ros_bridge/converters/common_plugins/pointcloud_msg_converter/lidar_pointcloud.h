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
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/converter_base/convert_ros_single.h"

#if __has_include("sensor_msgs/point_cloud2_iterator.hpp")
#include "sensor_msgs/point_cloud2_iterator.hpp"
#define ENABLE_ROS_MSG
#endif

#ifdef ENABLE_ROS_MSG
using InputMsg = sensor_msgs::msg::PointCloud2;
#else
using InputMsg = apollo::cyber::proto::SimpleMessage;
#endif

using OutputMsg = apollo::drivers::PointCloud;

using InputMsgPtr = std::shared_ptr<InputMsg>;
using OutputMsgPtr = std::shared_ptr<OutputMsg>;

namespace apollo {
namespace cyber {

class LidarPointcloud
    : public apollo::cyber::RosApolloMessageConverter<
          InputTypes<InputMsgPtr>, OutputTypes<OutputMsgPtr>> {
 public:
  LidarPointcloud() {}
  ~LidarPointcloud() {}

  /**
   * @brief convert the message between ros and apollo
   *
   * @param InputMsgPtr shared pointer of input message
   * @param OutputMsgPtr shared pointer of output message
   * @return result, true for success
   */
  virtual bool ConvertMsg(InputTypes<InputMsgPtr>&, OutputTypes<OutputMsgPtr>&);

  inline bool IsSystemBigEndian() {
    const uint32_t one = 0x01;
    return reinterpret_cast<const uint8_t*>(&one)[0] == 0;
  }

#ifdef ENABLE_ROS_MSG
  inline int FindFieldIndex(
      std::string name, sensor_msgs::msg::PointCloud2& cloud_msg) {  // NOLINT
    for (int i = 0; i < cloud_msg.fields.size(); i++) {
      if (cloud_msg.fields[i].name == name) {
        return i;
      }
    }
    return -1;
  }
#endif

  template <typename T>
  T ReadAsHostEndian(const bool is_bigendian, const uint8_t* data) {
    T value;
    std::memcpy(&value, data, sizeof(T));

    if ((is_bigendian && !IsSystemBigEndian()) ||
        (!is_bigendian && IsSystemBigEndian())) {
      std::reverse(reinterpret_cast<uint8_t*>(&value),
                   reinterpret_cast<uint8_t*>(&value) + sizeof(T));
    }

    return value;
  }
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::cyber::LidarPointcloud,
                                     apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
