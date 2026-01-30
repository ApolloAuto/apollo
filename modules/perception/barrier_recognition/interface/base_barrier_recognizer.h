/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "cyber/common/macros.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

struct BarrierRecognizerInitOptions : public BaseInitOptions {};

struct BarrierRecognizerOptions {
    std::string name;
    std::vector<double> world_roi_polygon;
};

struct BarrieGate {
    int status_id;
    std::string id;
    std::string type;
    float open_percent;
};


class BaseBarrierRecognizer {
public:
    BaseBarrierRecognizer() = default;

    virtual ~BaseBarrierRecognizer() = default;

    /**
     * @brief Init base barrier detector
     *
     * @param options
     * @return true
     * @return false
     */
    virtual bool Init(const BarrierRecognizerInitOptions& options = BarrierRecognizerInitOptions()) = 0;

    /**
     * @brief Recognize barrier gate status from point cloud.
     *
     * @param options
     * @param frame lidar frame
     * @return true
     * @return false
     */
    virtual bool Recognize(
            const BarrierRecognizerOptions& options,
            lidar::LidarFrame* frame,
            float& open_percent) = 0;
            
    /**
     * @brief Get class name
     *
     * @return std::string
     */
    virtual std::string Name() const = 0;

private:
    DISALLOW_COPY_AND_ASSIGN(BaseBarrierRecognizer);
};  // class BaseBarrierRecognizer

PERCEPTION_REGISTER_REGISTERER(BaseBarrierRecognizer);
#define PERCEPTION_REGISTER_CURB_DETECTOR(name) PERCEPTION_REGISTER_CLASS(BaseBarrierRecognizer, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
