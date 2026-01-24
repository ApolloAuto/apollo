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

#include "cyber/component/component.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h"
#include "modules/perception/traffic_light_detection/interface/base_traffic_light_detector.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::perception::onboard::TrafficDetectMessage;

class TrafficLightDetectComponent : public cyber::Component<TrafficDetectMessage> {
public:
    /**
     * @brief Construct a new traffic light detect component object.
     *
     */
    TrafficLightDetectComponent() = default;
    /**
     * @brief Destroy the traffic light detect component object.
     *
     */
    ~TrafficLightDetectComponent() = default;
    /**
     * @brief Initialize configuration files, algorithm plug-ins.
     *
     * @return true
     * @return false
     */
    bool Init() override;
    /**
     * @brief Trigger the detection process based on
              the traffic detect message.
     *
     * @param message
     * @return true
     * @return false
     */
    bool Proc(const std::shared_ptr<TrafficDetectMessage>& message) override;

private:
    int InitConfig();
    bool InitAlgorithmPlugin();
    bool InternalProc(const std::shared_ptr<TrafficDetectMessage const>& message);

private:
    trafficlight::TrafficLightDetectorInitOptions detector_init_options_;
    std::string tl_detector_name_;
    int gpu_id_;
    std::string config_path_;
    std::string config_file_;

    std::shared_ptr<trafficlight::BaseTrafficLightDetector> detector_;
    std::shared_ptr<apollo::cyber::Writer<TrafficDetectMessage>> writer_;
};

CYBER_REGISTER_COMPONENT(TrafficLightDetectComponent);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
