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

#include "modules/perception/traffic_light/rectify/unity_rectify.h"

#include "modules/common/util/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/rectify/cropbox.h"
#include "modules/perception/traffic_light/rectify/detection.h"
#include "modules/perception/traffic_light/rectify/select.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetAbsolutePath;

bool UnityRectify::Init() {
  ConfigManager *config_manager = ConfigManager::instance();
  if (config_manager == nullptr) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const ModelConfig *model_config = config_manager->GetModelConfig(name());
  if (model_config == nullptr) {
    AERROR << "not found model config: " << name();
    return false;
  }

  InitDetection(config_manager, model_config, &detect_, &crop_);

  select_.reset(new GaussianSelect);

  return true;
}

bool UnityRectify::InitDetection(const ConfigManager *config_manager,
                                 const ModelConfig *model_config,
                                 std::shared_ptr<IRefine> *detection,
                                 std::shared_ptr<IGetBox> *crop) {
  float crop_scale = 0.0;
  int crop_min_size = 0;
  int crop_method = 0;
  std::string detection_model;
  std::string detection_net;

  if (!model_config->GetValue("crop_scale", &crop_scale)) {
    AERROR << "crop_scale not found." << model_config->name();
    return false;
  }

  if (!model_config->GetValue("crop_min_size", &crop_min_size)) {
    AERROR << "crop_min_size not found." << model_config->name();
    return false;
  }

  if (!model_config->GetValue("crop_method", &crop_method)) {
    AERROR << "crop_method not found." << model_config->name();
    return false;
  }

  if (!model_config->GetValue("detection_model", &detection_model)) {
    AERROR << "detection_model not found." << model_config->name();
    return false;
  }
  detection_model =
      GetAbsolutePath(config_manager->WorkRoot(), detection_model);
  if (!model_config->GetValue("detection_net", &detection_net)) {
    AERROR << "detection_net not found." << model_config->name();
    return false;
  }
  detection_net = GetAbsolutePath(config_manager->WorkRoot(), detection_net);

  switch (crop_method) {
    default:
    case 0:
      crop->reset(new CropBox(crop_scale, crop_min_size));
      break;
    case 1:
      crop->reset(new CropBoxWholeImage());
      break;
  }
  int detect_method = 0;
  if (!model_config->GetValue("detect_method", &detect_method)) {
    AERROR << "detect_method not found." << model_config->name();
    return false;
  }
  switch (detect_method) {
    default:
    case 0:
      detection->reset(
          new Detection(crop_min_size, detection_net, detection_model));
      break;
    case 1:
      detection->reset(new DummyRefine());
      break;
  }

  return true;
}
bool UnityRectify::Rectify(const Image &image, const RectifyOption &option,
                           std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  std::vector<LightPtr> &lights_ref = *lights;
  std::vector<LightPtr> selected_bboxes;
  std::vector<LightPtr> detected_bboxes;

  for (auto &light : lights_ref) {
    // By default, the first debug ros is crop roi. (Reserve a position here).
    light->region.rectified_roi = light->region.projection_roi;
    light->region.debug_roi.push_back(cv::Rect(0, 0, 0, 0));
    light->region.debug_roi_detect_scores.push_back(0.0f);
  }

  cv::Rect cbox;
  crop_->GetCropBox(ros_image.size(), lights_ref, &cbox);
  AINFO << ros_image.size();
  AINFO << cbox;
  if (BoxIsValid(cbox, ros_image.size())) {
    lights_ref[0]->region.debug_roi[0] = cbox;

    detect_->SetCropBox(cbox);
    detect_->Perform(ros_image, &detected_bboxes);

    AINFO << "detect " << detected_bboxes.size() << " lights";
    for (size_t j = 0; j < detected_bboxes.size(); ++j) {
      AINFO << detected_bboxes[j]->region.rectified_roi;
      cv::Rect &region = detected_bboxes[j]->region.rectified_roi;
      float score = detected_bboxes[j]->region.detect_score;
      region.x += cbox.x;
      region.y += cbox.y;
      lights_ref[0]->region.debug_roi.push_back(region);
      lights_ref[0]->region.debug_roi_detect_scores.push_back(score);
    }

    select_->Select(ros_image, lights_ref, detected_bboxes, &selected_bboxes);
  } else {
    for (size_t h = 0; h < lights_ref.size(); ++h) {
      LightPtr light = lights_ref[h];
      light->region.is_detected = false;
      selected_bboxes.push_back(light);
    }
  }

  for (size_t i = 0; i < lights_ref.size(); ++i) {
    if (!selected_bboxes[i]->region.is_detected ||
        !selected_bboxes[i]->region.is_selected) {
      AWARN << "No detection box ,using project box";
    }
    cv::Rect region = selected_bboxes[i]->region.rectified_roi;
    lights_ref[i]->region.rectified_roi = region;
    lights_ref[i]->region.detect_class_id =
        selected_bboxes[i]->region.detect_class_id;
    lights_ref[i]->region.detect_score =
        selected_bboxes[i]->region.detect_score;
    lights_ref[i]->region.is_detected = selected_bboxes[i]->region.is_detected;
    lights_ref[i]->region.is_selected = selected_bboxes[i]->region.is_selected;
    AINFO << region;
  }
  return true;
}

std::string UnityRectify::name() const { return "UnityRectify"; }

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
