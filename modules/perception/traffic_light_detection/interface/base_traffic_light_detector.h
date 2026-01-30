/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace trafficlight {

struct TrafficLightDetectorInitOptions : public BaseInitOptions {
    std::shared_ptr<base::BaseCameraModel> base_camera_model = nullptr;
};

class BaseTrafficLightDetector {
public:
    /**
     * @brief Construct a new base traffic light detector object.
     *
     */
    BaseTrafficLightDetector() = default;
    /**
     * @brief Destroy the base traffic light detector object.
     *
     */
    virtual ~BaseTrafficLightDetector() = default;
    /**
     * @brief Initialize traffic light detector parameters.
     *
     * @param options
     * @return true
     * @return false
     */
    virtual bool Init(const TrafficLightDetectorInitOptions& options = TrafficLightDetectorInitOptions()) = 0;
    /**
     * @brief Detect traffic light from image.
     *
     * @param frame
     * @return true
     * @return false
     */
    virtual bool Detect(camera::TrafficLightFrame* frame) = 0;

    DISALLOW_COPY_AND_ASSIGN(BaseTrafficLightDetector);
};

PERCEPTION_REGISTER_REGISTERER(BaseTrafficLightDetector);
#define REGISTER_TRAFFIC_LIGHT_DETECTOR(name) PERCEPTION_REGISTER_CLASS(BaseTrafficLightDetector, name)

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
