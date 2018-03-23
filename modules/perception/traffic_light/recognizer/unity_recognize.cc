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
#include "modules/perception/traffic_light/recognizer/classify.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetAbsolutePath;

bool UnityRecognize::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  if (config_manager == nullptr) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const ModelConfig *model_config_night =
      config_manager->GetModelConfig(name() + "Night");
  if (model_config_night == nullptr) {
    AERROR << "not found model config: " << name() + "Night";
    return false;
  }
  if (!InitModel(config_manager, model_config_night, &classify_night_)) {
    AERROR << "init night model failed";
    return false;
  }

  const ModelConfig *model_config_day = config_manager->GetModelConfig(name());
  if (model_config_day == nullptr) {
    AERROR << "not found model config: " << name();
    return false;
  }

  if (!InitModel(config_manager, model_config_day, &classify_day_)) {
    AERROR << "init day model failed";
    return false;
  }

  return true;
}

bool UnityRecognize::InitModel(const ConfigManager *config_manager,
                               const ModelConfig *model_config,
                               std::shared_ptr<IRefine> *classify) {
  std::string classify_model;
  std::string classify_net;

  if (!model_config->GetValue("classify_model", &classify_model)) {
    AERROR << "classify_model not found." << name();
    return false;
  }
  classify_model = GetAbsolutePath(config_manager->WorkRoot(), classify_model);
  if (!model_config->GetValue("classify_net", &classify_net)) {
    AERROR << "classify_net not found." << name();
    return false;
  }
  classify_net = GetAbsolutePath(config_manager->WorkRoot(), classify_net);

  float classify_threshold = 0.0;
  int classify_resize_width = 0;
  int classify_resize_height = 0;

  if (!model_config->GetValue("classify_threshold", &classify_threshold)) {
    AERROR << "classify_threshold not found." << name();
    return false;
  }

  if (!model_config->GetValue("classify_resize_width",
                              &classify_resize_width)) {
    AERROR << "classify_resize_width not found." << name();
    return false;
  }
  if (!model_config->GetValue("classify_resize_height",
                              &classify_resize_height)) {
    AERROR << "classify_resize_height not found." << name();
    return false;
  }
  if (!model_config->GetValue("classify_threshold", &classify_threshold)) {
    AERROR << "classify_threshold not found." << name();
    return false;
  }
  classify->reset(new ClassifyBySimple(classify_net, classify_model,
                                       classify_threshold,
                                       (unsigned int)classify_resize_width,
                                       (unsigned int)classify_resize_height));
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
