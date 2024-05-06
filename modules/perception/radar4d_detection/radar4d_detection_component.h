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
#include <string>
#include <vector>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/perception/radar4d_detection/proto/radar4d_component_config.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/common/base/sensor_meta.h"
#include "modules/perception/common/base/radar_point_cloud.h"
#include "modules/perception/common/radar/common/radar_frame_pool.h"
#include "modules/perception/common/radar/common/radar_frame.h"
#include "modules/perception/common/hdmap/hdmap_input.h"
#include "modules/perception/common/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/common/onboard/msg_buffer/msg_buffer.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/radar4d_detection/app/radar_obstacle_perception.h"
#include "modules/perception/radar4d_detection/interface/base_preprocessor.h"
#include "modules/perception/radar4d_detection/interface/base_obstacle_perception.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::drivers::OculiiPointCloud;
using apollo::localization::LocalizationEstimate;

class Radar4dDetectionComponent : public
  cyber::Component<drivers::OculiiPointCloud> {
 public:
  Radar4dDetectionComponent()
      : tf_child_frame_id_(""),
        radar_forward_distance_(200.0),
        odometry_channel_name_(""),
        hdmap_input_(nullptr),
        radar_preprocessor_(nullptr),
        radar_perception_(nullptr) {}
  ~Radar4dDetectionComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::OculiiPointCloud>& message) override;

 private:
  bool InitAlgorithmPlugin(const Radar4dDetectionConfig& config);
  bool InternalProc(
    const std::shared_ptr<const drivers::OculiiPointCloud>& in_message,
    std::shared_ptr<onboard::SensorFrameMessage> out_message);
  bool GetCarLocalizationSpeed(double timestamp,
                               Eigen::Vector3f* car_linear_speed,
                               Eigen::Vector3f* car_angular_speed);

  Radar4dDetectionComponent(
    const Radar4dDetectionComponent&) = delete;
  Radar4dDetectionComponent& operator=(
    const Radar4dDetectionComponent&) = delete;

 private:
  static std::atomic<uint32_t> seq_num_;

  base::SensorInfo radar_info_;
  std::string tf_child_frame_id_;
  double radar_forward_distance_;
  std::string odometry_channel_name_;

  onboard::TransformWrapper radar2world_trans_;
  onboard::TransformWrapper radar2novatel_trans_;
  map::HDMapInput* hdmap_input_;
  std::shared_ptr<BasePreprocessor> radar_preprocessor_;
  std::shared_ptr<BaseRadarObstaclePerception> radar_perception_;
  onboard::MsgBuffer<LocalizationEstimate> localization_subscriber_;
  std::shared_ptr<apollo::cyber::Writer<onboard::SensorFrameMessage>> writer_;
};

CYBER_REGISTER_COMPONENT(Radar4dDetectionComponent);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
