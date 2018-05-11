/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/traffic_light/recognizer/unity_recognize.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/traffic_light/recognizer/classify.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;

bool UnityRecognize::Init() {
  if (!GetProtoFromFile(FLAGS_traffic_light_recognizer_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_recognizer_config;
    return false;
  }

  if (config_.recognizer_config_size() != 2) {
    AERROR << "RecognizeConfig size should be 2.";
    return false;
  }
  for (const auto &recognizer_config : config_.recognizer_config()) {
    if (recognizer_config.name() == "UnityRecognizeNight") {
      classify_night_ = std::make_shared<ClassifyBySimple>(
          recognizer_config.classify_net(), recognizer_config.classify_model(),
          recognizer_config.classify_threshold(),
          static_cast<unsigned int>(recognizer_config.classify_resize_width()),
          static_cast<unsigned int>(
              recognizer_config.classify_resize_height()));
    }
    if (recognizer_config.name() == "UnityRecognize") {
      classify_day_ = std::make_shared<ClassifyBySimple>(
          recognizer_config.classify_net(), recognizer_config.classify_model(),
          recognizer_config.classify_threshold(),
          static_cast<unsigned int>(recognizer_config.classify_resize_width()),
          static_cast<unsigned int>(
              recognizer_config.classify_resize_height()));
    }
  }
  return true;
}

bool UnityRecognize::RecognizeStatus(const Image &image,
                                     const RecognizeOption &option,
                                     std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  cv::Rect cbox;
  cbox = cv::Rect(0, 0, ros_image.cols, ros_image.rows);
  classify_night_->SetCropBox(cbox);
  classify_day_->SetCropBox(cbox);
  std::vector<LightPtr> candidate(1);
  for (LightPtr light : *lights) {
    if (light->region.is_detected) {
      candidate[0] = light;
      if (light->region.detect_class_id == QUADRATE_CLASS) {
        AINFO << "Recognize Use Night Model!";
        classify_night_->Perform(ros_image, &candidate);
      } else if (light->region.detect_class_id == VERTICAL_CLASS) {
        AINFO << "Recognize Use Day Model!";
        classify_day_->Perform(ros_image, &candidate);
      } else {
        AINFO << "Not support yet!";
      }
    } else {
      light->status.color = UNKNOWN_COLOR;
      light->status.confidence = 0;
      AINFO << "Unknown Detection Class: " << light->region.detect_class_id
            << ". region.is_detected: " << light->region.is_detected
            << ". Not perform recognition.";
    }
  }
  return true;
}

std::string UnityRecognize::name() const { return "UnityRecognize"; }

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
