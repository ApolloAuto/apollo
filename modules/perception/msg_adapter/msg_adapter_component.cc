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
#include "modules/perception/msg_adapter/msg_adapter_component.h"

#include "modules/perception/msg_adapter/common/msg_adapter_gflags.h"
#include "modules/perception/msg_adapter/convert/convert.h"

namespace apollo {
namespace perception {

bool MsgAdapterComponent::Init() {
  msg_converter_ = std::make_shared<MsgConverter>(node_);
  // register message conversion
  msg_converter_->Add(FLAGS_cameraframe_to_obstacles_in,
                      FLAGS_cameraframe_to_obstacles_out,
                      ConvertCameraFrame2Obstacles);
  AINFO << "cameraframe_to_obstacles_in: "
        << FLAGS_cameraframe_to_obstacles_in
        << " cameraframe_to_obstacles_out: "
        << FLAGS_cameraframe_to_obstacles_out;

  msg_converter_->Add(FLAGS_sensorframe_message_to_obstacles_in,
                      FLAGS_sensorframe_message_to_obstacles_out,
                      ConvertSensorFrameMessage2Obstacles);
  AINFO << "sensorframe_message_to_obstacles_in: "
        << FLAGS_sensorframe_message_to_obstacles_in
        << " sensorframe_message_to_obstacles_out: "
        << FLAGS_sensorframe_message_to_obstacles_out;

  msg_converter_->Add(FLAGS_lidarframe_to_obstacles_in,
                      FLAGS_lidarframe_to_obstacles_out,
                      ConvertLidarFrameMessage2Obstacles);
  AINFO << "lidarframe_to_obstacles_in: "
        << FLAGS_lidarframe_to_obstacles_in
        << " lidarframe_to_obstacles_out: "
        << FLAGS_lidarframe_to_obstacles_out;

  return true;
}

}  // namespace perception
}  // namespace apollo
