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
#ifndef PERCEPTION_ONBOARD_COMPONENT_FUSION_COMPONENT_H_
#define PERCEPTION_ONBOARD_COMPONENT_FUSION_COMPONENT_H_

#include <memory>
#include <mutex>  // NOLINT
#include <string>
#include <vector>

#include "cybertron/component/component.h"
#include "modules/perception/base/object.h"
#include "modules/perception/fusion/app/obstacle_multi_sensor_fusion.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace onboard {

class FusionComponent : public cybertron::Component<SensorFrameMessage> {
 public:
  FusionComponent() = default;
  ~FusionComponent() = default;
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(const std::shared_ptr<SensorFrameMessage const>& in_message,
                    std::shared_ptr<PerceptionObstacles> out_message,
                    std::shared_ptr<SensorFrameMessage> viz_message);
  bool BuildSensorObjs(const std::shared_ptr<SensorFrameMessage const>& message,
                       base::FramePtr& multi_sensor_objs) const;

 private:
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;
  std::unique_ptr<fusion::ObstacleMultiSensorFusion> fusion_;
  map::HDMapInput* hdmap_input_ = nullptr;
  std::shared_ptr<apollo::cybertron::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cybertron::Writer<SensorFrameMessage>> inner_writer_;
};

CYBERTRON_REGISTER_COMPONENT(FusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_ONBOARD_COMPONENT_FUSION_COMPONENT_H_
