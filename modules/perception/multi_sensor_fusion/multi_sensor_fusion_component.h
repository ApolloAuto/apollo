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

#include "modules/perception/multi_sensor_fusion/proto/fusion_component_config.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/hdmap/hdmap_input.h"
#include "modules/perception/common/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/multi_sensor_fusion/interface/base_fusion_system.h"

namespace apollo {
namespace perception {
namespace fusion {

using onboard::SensorFrameMessage;

class MultiSensorFusionComponent : public cyber::Component<SensorFrameMessage> {
 public:
  MultiSensorFusionComponent() = default;
  ~MultiSensorFusionComponent() = default;
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  bool InitAlgorithmPlugin(const FusionComponentConfig &config);
  bool InternalProc(const std::shared_ptr<SensorFrameMessage const>& in_message,
                    std::shared_ptr<PerceptionObstacles> out_message,
                    std::shared_ptr<SensorFrameMessage> viz_message);

 private:
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;

  bool object_in_roi_check_ = false;
  double radius_for_roi_object_check_ = 0;

  std::unique_ptr<fusion::BaseFusionSystem> fusion_;
  map::HDMapInput* hdmap_input_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> inner_writer_;
};

CYBER_REGISTER_COMPONENT(MultiSensorFusionComponent);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
