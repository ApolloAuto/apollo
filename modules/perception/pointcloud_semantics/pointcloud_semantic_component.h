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

#include <string>
#include <memory>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/pointcloud_semantics/proto/pointcloud_semantic_component_config.pb.h"
#include "modules/perception/pointcloud_semantics/interface/base_pointcloud_parser.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;

class PointCloudSemanticComponent final : public cyber::Component<LidarFrameMessage> {
public:
    PointCloudSemanticComponent() {}
    virtual ~PointCloudSemanticComponent() {}
    /**
     * @brief Init pointcloud semantic component
     *
     * @return true
     * @return false
     */
    bool Init() override;
    /**
     * @brief Process of pointcloud semantic component
     *
     * @param message lidar frame message
     * @return true
     * @return false
     */
    bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

private:
    /**
     * @brief InternalProc, main function is defined here.
     * @param LidarFrameMessage, input/output message
     * @return true if success
     */
    bool InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message);

private:
    std::string output_channel_name_;
    std::shared_ptr<apollo::cyber::Writer<LidarFrameMessage>> writer_;
    std::shared_ptr<BasePointCloudParser> parser_;
};

CYBER_REGISTER_COMPONENT(PointCloudSemanticComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
