/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#ifndef PERCEPTION_ONBOARD_LIDAR_ONBOARD_INNER_COMPONENT_MESSAGES_H_
#define PERCEPTION_ONBOARD_LIDAR_ONBOARD_INNER_COMPONENT_MESSAGES_H_

#include <string>
#include "cybertron/cybertron.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/proto/perception_obstacle.pb.h"  // NOLINT

namespace apollo {
namespace perception {
namespace onboard {

class LidarFrameMessage : public apollo::cybertron::message::IntraMessage {
 public:
  LidarFrameMessage() : lidar_frame_(nullptr) {
    type_name_ = "LidarFrameMessage";
  }

  ~LidarFrameMessage() = default;

  std::string GetTypeName() const { return type_name_; }

  LidarFrameMessage* New() const { return new LidarFrameMessage; }

  bool SerializeToString(std::string* str) const { return false; }
  bool ParseFromString(const std::string& str) { return false; }

  static Descriptor* descriptor() { return new Descriptor(); }

 public:
  double timestamp_ = 0.0;
  uint32_t seq_num_ = 0;
  ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
  std::shared_ptr<lidar::LidarFrame> lidar_frame_;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_ONBOARD_LIDAR_ONBOARD_INNER_COMPONENT_MESSAGES_H_
