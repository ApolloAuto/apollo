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
#ifndef PERCEPTION_ONBOARD_COMPONENT_RECOGNITION_COMPONENT_H_
#define PERCEPTION_ONBOARD_COMPONENT_RECOGNITION_COMPONENT_H_

#include "cybertron/cybertron.h"
#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/lidar/app/lidar_obstacle_tracking.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace onboard {

class RecognitionComponent : public cybertron::Component<LidarFrameMessage> {
 public:
  RecognitionComponent() : tracker_(nullptr) {}
  ~RecognitionComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(const std::shared_ptr<const LidarFrameMessage>& in_message,
                    const std::shared_ptr<SensorFrameMessage>& out_message);
  std::unique_ptr<lidar::LidarObstacleTracking> tracker_;
  base::SensorInfo sensor_info_;
  std::shared_ptr<apollo::cybertron::Writer<SensorFrameMessage>> writer_;
};

CYBERTRON_REGISTER_COMPONENT(RecognitionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_ONBOARD_COMPONENT_RECOGNITION_COMPONENT_H_
