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
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/traffic_light_recognition/interface/base_traffic_light_recognitor.h"
#include "modules/perception/traffic_light_recognition/recognition/caffe_recognizer/classify.h"
#include "modules/perception/traffic_light_recognition/recognition/caffe_recognizer/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace trafficlight {

class TrafficLightRecognition final : public BaseTrafficLightRecognitor {
public:
    /**
     * @brief Construct a new Traffic Light Recognition object
     *
     */
    TrafficLightRecognition() = default;
    /**
     * @brief Destroy the Traffic Light Recognition object
     *
     */
    ~TrafficLightRecognition() = default;
    /**
     * @brief Initialize traffic light recognitor parameters.
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const TrafficLightRecognitorInitOptions& options) override;

    /**
     * @brief recogn traffic_light from image.
     *
     * @param frame
     * @return true
     * @return false
     */
    bool Detect(camera::TrafficLightFrame* frame) override;

private:
    std::shared_ptr<ClassifyBySimple> classify_vertical_;
    std::shared_ptr<ClassifyBySimple> classify_quadrate_;
    std::shared_ptr<ClassifyBySimple> classify_horizontal_;

    TrafficLightRecognitionConfig recognize_param_;
    std::string recognition_root_dir;

    DISALLOW_COPY_AND_ASSIGN(TrafficLightRecognition);
};

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
