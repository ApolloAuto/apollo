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
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/rectify/cropbox.h"
#include "modules/perception/traffic_light/rectify/detection.h"
#include "modules/perception/traffic_light/rectify/select.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using apollo::common::util::GetProtoFromFile;

bool UnityRectify::Init() {
  if (!GetProtoFromFile(FLAGS_traffic_light_rectifier_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_traffic_light_rectifier_config;
    return false;
  }

  switch (config_.crop_method()) {
    default:
    case 0:
      crop_ = std::make_shared<CropBox>(config_.crop_scale(),
                                        config_.crop_min_size());
      break;
    case 1:
      crop_ = std::make_shared<CropBoxWholeImage>();
      break;
  }
  switch (config_.detect_method()) {
    default:
    case 0:
      detect_ = std::make_shared<Detection>(config_.crop_min_size(),
                                            config_.detection_net(),
                                            config_.detection_model());
      break;
    case 1:
      detect_ = std::make_shared<DummyRefine>();
      break;
  }

  select_ = std::make_shared<GaussianSelect>();

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
