//
// Created by gaohan02 on 16-9-20.
//

#include "module/perception/traffic_light/recognizer/color_recognize/color_recognize.h"
#include "module/perception/traffic_light/recognizer/color_recognize/classify.h"
#include <lib/base/file_util.h>
#include <traffic_light/base/utils.h>

namespace adu {
namespace perception {
namespace traffic_light {

bool ColorRecognize::init() {

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

  _night_begin = hour_minute_to_second(FLAGS_night_begin_hour, FLAGS_night_begin_minute);
  _night_end = hour_minute_to_second(FLAGS_night_end_hour, FLAGS_night_end_minute);

  return true;
}
bool ColorRecognize::init_model(const config_manager::ConfigManager *config_manager,
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
  float classify_data_scale = 0;
  int classify_resize_width = 0;
  int classify_resize_height = 0;
  std::vector<float> classify_rgb_mean;

  if (!model_config->get_value("classify_threshold", &classify_threshold)) {
    AERROR << "classify_threshold not found." << name();
    return false;
  }

  if (!model_config->get_value("classify_data_scale", &classify_data_scale)) {
    AERROR << "classify_data_scale not found." << name();
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
  if (!model_config->get_value(\
            "classify_rgb_mean", &classify_rgb_mean)) {
    AERROR << "classify_rgb_mean not found." << name();
    return false;
  }
  classify->reset(new ClassifyByDenseBoxMulti(classify_net,
                                              classify_model,
                                              classify_threshold,
                                              classify_data_scale,
                                              (unsigned int) classify_resize_width,
                                              (unsigned int) classify_resize_height,
                                              &classify_rgb_mean[0]));
  return true;
}

bool ColorRecognize::recognize_status(const Image &image, const RecognizeOption &option,
                                      std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  std::vector<LightPtr> &lights_ref = *lights;
  std::vector<BoundBox_t> green_bboxes;
  for (int i = 0; i < lights_ref.size(); ++i) {
    BoundBox_t box;
    box.isValid = true;
    box.selected = lights_ref[i]->region.is_detected;
    box.rect = lights_ref[i]->region.rectified_roi;
    green_bboxes.push_back(box);
  }

  BoundBox_t cbox;
  cbox.isValid = true;
  cbox.rect = cv::Rect(0, 0, ros_image.cols, ros_image.rows);
  int sec = (int(image.ts())) % (24 * 3600);
  int hour = 0;
  int minute = 0;
  get_hour_minute(image.ts(), hour, minute);
  AINFO << "Image time: " << hour + 8 << ":" << minute;
  if (_night_begin < sec && sec < _night_end) {
    AINFO << "Recognize Use Night Model!";
    _classify_night->set_crop_box(cbox);
    _classify_night->perform(ros_image, green_bboxes);
  } else {
    AINFO << "Recognize Use Day Model!";
    _classify_day->set_crop_box(cbox);
    _classify_day->perform(ros_image, green_bboxes);
  }
  for (int i = 0; i < lights_ref.size(); ++i) {
    if (!lights_ref[i]->region.is_detected) {
      lights_ref[i]->status.color = UNKNOWN_COLOR;
      lights_ref[i]->status.confidence = 0;
    } else {
      lights_ref[i]->status.color = green_bboxes[i].light_status;
      lights_ref[i]->status.confidence = green_bboxes[i].probability;
    }
  }
  return true;
}

std::string ColorRecognize::name() const {
  return "ColorRecognize";
}

REGISTER_RECOGNIZER(ColorRecognize);
}
}
}
