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
#include "modules/perception/onboard/component/lidar_output_component.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

namespace apollo {
namespace perception {
namespace onboard {

bool LidarOutputComponent::Init() {
  writer_ =
      node_->CreateWriter<PerceptionObstacles>("/apollo/perception/obstacles");
  return true;
}

bool LidarOutputComponent::Proc(
    const std::shared_ptr<SensorFrameMessage>& message) {
  std::shared_ptr<PerceptionObstacles> out_message(new PerceptionObstacles);

  if (message->frame_ == nullptr) {
    AERROR << "Failed to get frame in message.";
    return false;
  }

  if (!MsgSerializer::SerializeMsg(
          message->timestamp_, message->lidar_timestamp_, message->seq_num_,
          message->frame_->objects, message->error_code_, out_message.get())) {
    AERROR << "Failed to serialize PerceptionObstacles object.";
    return false;
  }

  writer_->Write(out_message);
  // Send("/apollo/perception/obstacles", out_message);

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
