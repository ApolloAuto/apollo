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

#include "modules/perception/traffic_light_recognition/recognition/caffe_recognizer/recognition.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace trafficlight {

bool TrafficLightRecognition::Init(const TrafficLightRecognitorInitOptions& options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);

    if (!cyber::common::GetProtoFromFile(config_file, &recognize_param_)) {
        AINFO << "load proto param failed, root dir: " << config_file;
        return false;
    }

    classify_quadrate_.reset(new ClassifyBySimple);
    classify_vertical_.reset(new ClassifyBySimple);
    classify_horizontal_.reset(new ClassifyBySimple);

    classify_quadrate_->Init(recognize_param_.quadrate_model(), options.gpu_id);
    classify_vertical_->Init(recognize_param_.vertical_model(), options.gpu_id);
    classify_horizontal_->Init(recognize_param_.horizontal_model(), options.gpu_id);

    return true;
}

bool TrafficLightRecognition::Detect(camera::TrafficLightFrame* frame) {
    std::vector<base::TrafficLightPtr> candidate(1);

    for (base::TrafficLightPtr light : frame->traffic_lights) {
        if (light->region.is_detected) {
            candidate[0] = light;
            if (light->region.detect_class_id == base::TLDetectionClass::TL_QUADRATE_CLASS) {
                AINFO << "Recognize Use Quadrate Model!";
                classify_quadrate_->Perform(frame, &candidate);
            } else if (light->region.detect_class_id == base::TLDetectionClass::TL_VERTICAL_CLASS) {
                AINFO << "Recognize Use Vertical Model!";
                classify_vertical_->Perform(frame, &candidate);
            } else if (light->region.detect_class_id == base::TLDetectionClass::TL_HORIZONTAL_CLASS) {
                AINFO << "Recognize Use Horizonal Model!";
                classify_horizontal_->Perform(frame, &candidate);
            } else {
                return false;
            }
        } else {
            light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
            light->status.confidence = 0;
        }
    }

    return true;
}

REGISTER_TRAFFIC_LIGHT_DETECTOR(TrafficLightRecognition);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
