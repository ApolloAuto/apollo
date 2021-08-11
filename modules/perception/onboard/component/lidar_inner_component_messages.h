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
#pragma once

#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

class LidarFrameMessage {
 public:
  LidarFrameMessage() : lidar_frame_(nullptr) {
    type_name_ = "LidarFrameMessage";
  }

  ~LidarFrameMessage() = default;

  std::string GetTypeName() const { return type_name_; }

  LidarFrameMessage* New() const { return new LidarFrameMessage; }

 public:
  double timestamp_ = 0.0;
  uint64_t lidar_timestamp_ = 0;
  uint32_t seq_num_ = 0;
  std::string type_name_;
  ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
  std::shared_ptr<lidar::LidarFrame> lidar_frame_;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
