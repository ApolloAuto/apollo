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

#include <atomic>
#include <limits>
#include <memory>
#include <string>

#include "modules/perception/lidar_segmentation/proto/lidar_segmentation_component_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/lidar_detection/interface/base_lidar_detector.h"
#include "modules/perception/lidar_segmentation/common/semantic_builder.h"

namespace apollo {
namespace perception {
namespace lidar {

class LidarSegmentationComponent : public cyber::Component<onboard::LidarFrameMessage> {
public:
    LidarSegmentationComponent() = default;
    virtual ~LidarSegmentationComponent() = default;
    /**
     * @brief Init lidar segment component
     *
     * @return true
     * @return false
     */
    bool Init() override;
    /**
     * @brief Process of lidar segmentation
     *
     * @param message lidar frame message
     * @return true
     * @return false
     */
    bool Proc(const std::shared_ptr<onboard::LidarFrameMessage>& message) override;

private:
    bool InternalProc(const std::shared_ptr<onboard::LidarFrameMessage>& message);

private:
    std::string output_channel_name_;
    std::string segmentor_name_;
    std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;

    // segmentation
    std::shared_ptr<BaseLidarDetector> segmentor_;
    // semantic builder
    SemanticBuilder semantic_builder_;
};

CYBER_REGISTER_COMPONENT(LidarSegmentationComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
