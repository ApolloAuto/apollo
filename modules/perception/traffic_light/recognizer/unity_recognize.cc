//
// Created by gaohan02 on 16-9-20.
//

#include "unity_recognize.h"
#include "classify.h"
#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool UnityRecognize::Init() {

  ConfigManager *config_manager = \
            ConfigManager::instance();
  if (config_manager == NULL) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const ModelConfig *model_config_night = NULL;
  if (!config_manager->GetModelConfig(name() + "Night", &model_config_night)) {
    AERROR << "not found model config: " << name() + "Night";
    return false;
  }
  if (!InitModel(config_manager, model_config_night, &classify_night_)) {
    AERROR << "init night model failed";
    return false;
  }

  const ModelConfig *model_config_day = NULL;
  if (!config_manager->GetModelConfig(name(), &model_config_day)) {
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

  if (!model_config->GetValue(\
            "classify_model", &classify_model)) {
    AERROR << "classify_model not found." << name();
    return false;
  }
  classify_model = FileUtil::GetAbsolutePath(config_manager->work_root(),
                                             classify_model);
  if (!model_config->GetValue("classify_net", &classify_net)) {
    AERROR << "classify_net not found." << name();
    return false;
  }
  classify_net = FileUtil::GetAbsolutePath(config_manager->work_root(),
                                           classify_net);

  float classify_threshold = 0;
  int classify_resize_width = 0;
  int classify_resize_height = 0;
  std::vector<float> classify_rgb_mean;

  if (!model_config->GetValue("classify_threshold", &classify_threshold)) {
    AERROR << "classify_threshold not found." << name();
    return false;
  }

  if (!model_config->GetValue("classify_resize_width", &classify_resize_width)) {
    AERROR << "classify_resize_width not found." << name();
    return false;
  }
  if (!model_config->GetValue("classify_resize_height", &classify_resize_height)) {
    AERROR << "classify_resize_height not found." << name();
    return false;
  }
  if (!model_config->GetValue(\
            "classify_threshold", &classify_threshold)) {
    AERROR << "classify_threshold not found." << name();
    return false;
  }
  classify->reset(new ClassifyBySimple(classify_net,
                                       classify_model,
                                       classify_threshold,
                                       (unsigned int) classify_resize_width,
                                       (unsigned int) classify_resize_height));
  return true;
}

bool UnityRecognize::RecognizeStatus(const Image &image, const RecognizeOption &option,
                                     std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  std::vector<LightPtr> &lights_ref = *lights;
  cv::Rect cbox;
  cbox = cv::Rect(0, 0, ros_image.cols, ros_image.rows);
  classify_night_->SetCropBox(cbox);
  classify_day_->SetCropBox(cbox);
  std::vector<LightPtr> candidate(1);
  for (LightPtr light:*lights) {
    if (light->region.is_detected) {
      candidate[0] = light;
      if (light->region.detect_class_id == QUADRATE_CLASS) {    // QUADRATE_CLASS (Night)
        AINFO << "Recognize Use Night Model!";
        classify_night_->Perform(ros_image, &candidate);
      } else if (light->region.detect_class_id == VERTICAL_CLASS) {    // VERTICAL_CLASS (Day)
        AINFO << "Recognize Use Day Model!";
        classify_day_->Perform(ros_image, &candidate);
      } else {

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

std::string UnityRecognize::name() const {
  return "UnityRecognize";
}

}
}
}
