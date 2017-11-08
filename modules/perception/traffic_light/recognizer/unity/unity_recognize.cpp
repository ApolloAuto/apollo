//
// Created by gaohan02 on 16-9-20.
//

#include "module/perception/traffic_light/recognizer/unity/unity_recognize.h"
#include "module/perception/traffic_light/recognizer/unity/classify.h"
#include <lib/base/file_util.h>
#include <traffic_light/base/utils.h>

namespace adu {
namespace perception {
namespace traffic_light {

bool UnityRecognize::init() {

  config_manager::ConfigManager *config_manager = \
            base::Singleton<config_manager::ConfigManager>::get();
  if (config_manager == NULL) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  const config_manager::ModelConfig *model_config_night = NULL;
  if (!config_manager->get_model_config(name() + "Night", &model_config_night)) {
    AERROR << "not found model config: " << name() + "Night";
    return false;
  }
  if (!init_model(config_manager, model_config_night, &_classify_night)) {
    AERROR << "init night model failed";
    return false;
  }

  const config_manager::ModelConfig *model_config_day = NULL;
  if (!config_manager->get_model_config(name(), &model_config_day)) {
    AERROR << "not found model config: " << name();
    return false;
  }

  if (!init_model(config_manager, model_config_day, &_classify_day)) {
    AERROR << "init day model failed";
    return false;
  }

  return true;
}
bool UnityRecognize::init_model(const config_manager::ConfigManager *config_manager,
                                const config_manager::ModelConfig *model_config,
                                std::shared_ptr<IRefine> *classify) {
  std::string classify_model;
  std::string classify_net;

  if (!model_config->get_value(\
            "classify_model", &classify_model)) {
    AERROR << "classify_model not found." << name();
    return false;
  }
  classify_model = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                     classify_model);
  if (!model_config->get_value(\
            "classify_net", &classify_net)) {
    AERROR << "classify_net not found." << name();
    return false;
  }
  classify_net = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                   classify_net);

  float classify_threshold = 0;
  int classify_resize_width = 0;
  int classify_resize_height = 0;
  std::vector<float> classify_rgb_mean;

  if (!model_config->get_value("classify_threshold", &classify_threshold)) {
    AERROR << "classify_threshold not found." << name();
    return false;
  }

  if (!model_config->get_value("classify_resize_width", &classify_resize_width)) {
    AERROR << "classify_resize_width not found." << name();
    return false;
  }
  if (!model_config->get_value("classify_resize_height", &classify_resize_height)) {
    AERROR << "classify_resize_height not found." << name();
    return false;
  }
  if (!model_config->get_value(\
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

bool UnityRecognize::recognize_status(const Image &image, const RecognizeOption &option,
                                      std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  std::vector<LightPtr> &lights_ref = *lights;
  cv::Rect cbox;
  cbox = cv::Rect(0, 0, ros_image.cols, ros_image.rows);
  _classify_night->set_crop_box(cbox);
  _classify_day->set_crop_box(cbox);
  std::vector<LightPtr> candidate(1);
  for (LightPtr light:*lights) {
    if (light->region.is_detected) {
      candidate[0] = light;
      if (light->region.detect_class_id == QUADRATE_CLASS) {    // QUADRATE_CLASS (Night)
        AINFO << "Recognize Use Night Model!";
        _classify_night->perform(ros_image, &candidate);
      } else if (light->region.detect_class_id == VERTICAL_CLASS) {    // VERTICAL_CLASS (Day)
        AINFO << "Recognize Use Day Model!";
        _classify_day->perform(ros_image, &candidate);
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

REGISTER_RECOGNIZER(UnityRecognize);
}
}
}
