//
// Created by gaohan02 on 16-9-13.
//

#include <lib/base/file_util.h>
#include <traffic_light/base/utils.h>
#include "module/perception/traffic_light/rectify/densebox/crop/verify_hdmap.h"
#include "module/perception/traffic_light/rectify/densebox/crop/cropbox.h"
#include "module/perception/traffic_light/rectify/densebox/detection/detection.h"
#include "module/perception/traffic_light/rectify/densebox/selector/rectify_hdmap.h"
#include "module/perception/traffic_light/rectify/densebox/selector/select_light.h"
#include "module/perception/traffic_light/rectify/densebox/densebox_rectify.h"
#include "lib/config_manager/config_manager.h"

namespace adu {
namespace perception {
namespace traffic_light {

bool DenseBoxRectify::init() {
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

  init_detection(config_manager, model_config_night, &_detect_night, &_crop_night);

  const config_manager::ModelConfig *model_config = NULL;
  if (!config_manager->get_model_config(name(), &model_config)) {
    AERROR << "not found model config: " << name();
    return false;
  }

  init_detection(config_manager, model_config, &_detect_day, &_crop_day);
  float iou_thresh = 0;
  float vert_thresh = 0;

  if (!model_config->get_value(\
            "iou_thresh", &iou_thresh)) {
    AERROR << "iou_thresh not found." << name();
    return false;
  }

  if (!model_config->get_value(\
            "vert_thresh", &vert_thresh)) {
    AERROR << "vert_thresh not found." << name();
    return false;
  }
  int enable_track = 0;
  if (!model_config->get_value(\
            "enable_track", &enable_track)) {
    AERROR << "enable_track not found." << name();
  }

  float hd_scale = 0;
  if (!model_config->get_value(\
            "hdmap_box_scale", &hd_scale)) {
    AERROR << "hdmap_box_scale not found." << name();
    return false;
  }
  _enable_track = (enable_track != 0);
  _verifymap.reset(new HDmapVerify(hd_scale));
  _rectify.reset(new RectifyHDmap(vert_thresh, iou_thresh));
  _select.reset(new SelectLight(iou_thresh));
  _kf.resize(2);
  _init.resize(2);
  for (int i = 0; i < _kf.size(); ++i) {
    _kf[i].init(2, 2);
    _kf[i].transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1);
    _kf[i].measurementMatrix = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1);
    _init[i] = false;
  }

  _night_begin = hour_minute_to_second(FLAGS_night_begin_hour, FLAGS_night_begin_minute);
  _night_end = hour_minute_to_second(FLAGS_night_end_hour, FLAGS_night_end_minute);

  return true;
}

bool DenseBoxRectify::init_detection(const config_manager::ConfigManager *config_manager,
                                     const config_manager::ModelConfig *model_config,
                                     std::shared_ptr<IRefine> *detection,
                                     std::shared_ptr<IGetBox> *crop) {

  float crop_scale = 0;
  int crop_min_size = 0;
  int crop_method = 0;

  if (!model_config->get_value(\
            "crop_scale", &crop_scale)) {
    AERROR << "crop_scale not found." << model_config->name();
    return false;
  }

  if (!model_config->get_value(\
            "crop_min_size", &crop_min_size)) {
    AERROR << "crop_min_size not found." << model_config->name();
    return false;
  }

  if (!model_config->get_value(\
            "crop_method", &crop_method)) {
    AERROR << "crop_method not found." << model_config->name();
    return false;
  }

  std::string detection_model;
  std::string detection_net;
  float nms_overlap = 0;

  if (!model_config->get_value(\
            "detection_model", &detection_model)) {
    AERROR << "detection_model not found." << model_config->name();
    return false;
  }
  detection_model = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                      detection_model);
  if (!model_config->get_value(\
            "detection_net", &detection_net)) {
    AERROR << "detection_net not found." << model_config->name();
    return false;
  }
  detection_net = base::FileUtil::get_absolute_path(config_manager->work_root(),
                                                    detection_net);
  if (!model_config->get_value(\
            "nms_overlap", &nms_overlap)) {
    AERROR << "nms_overlap not found." << model_config->name();
    return false;
  }

  switch (crop_method) {
    default:
    case 0:crop->reset(new CropBox(crop_scale, crop_min_size));
      break;
    case 1:crop->reset(new CropBoxWholeImage());
      break;
  }
  detection->reset(new DenseboxDetection(crop_min_size, detection_net, detection_model,
                                         nms_overlap));
  return true;
}
bool DenseBoxRectify::rectify(const Image &image, const RectifyOption &option,
                              std::vector<LightPtr> *lights) {
  cv::Mat ros_image = image.mat();
  std::vector<LightPtr> &lights_ref = *lights;
  std::vector<BoundBox_t> hdmap_bboxes;
  std::vector<BoundBox_t> green_bboxes;
  std::vector<BoundBox_t> refined_bboxes;

  for (int i = 0; i < lights_ref.size(); ++i) {
    BoundBox_t box;
    box.rect = lights_ref[i]->region.projection_roi;
    // 默认第一个debug roi 是 crop roi,这里先站位
    lights_ref[i]->region.debug_roi.push_back(cv::Rect(0, 0, 0, 0));
    if (_enable_track && _init[option.camera_id]) {
      cv::Mat predict = _kf[option.camera_id].predict();
      box.rect.x += predict.at<float>(0);
      box.rect.y += predict.at<float>(1);
      lights_ref[i]->region.debug_roi.push_back(box.rect);
    }
    hdmap_bboxes.push_back(box);
  }

  _verifymap->verify(ros_image, hdmap_bboxes);

  BoundBox_t cbox;
  _crop_day->get_crop_box(ros_image, hdmap_bboxes, cbox);

  if (cbox.isValid) {
    lights_ref[0]->region.debug_roi[0] = cbox.rect;
    int sec = (int(image.ts())) % (24 * 3600);
    int hour = 0;
    int minute = 0;
    get_hour_minute(image.ts(), hour, minute);
    AINFO << "Image time: " << hour + 8 << ":" << minute;
    if (_night_begin < sec && sec < _night_end) {
      AINFO << "Detect Use Night Model!";
      _detect_night->set_crop_box(cbox);
      _detect_night->perform(ros_image, refined_bboxes);
    } else {
      AINFO << "Detect Use day Model!";
      _detect_day->set_crop_box(cbox);
      _detect_day->perform(ros_image, refined_bboxes);
    }
    AINFO << "detect " << refined_bboxes.size() << " lights";
    for (int i = 0; i < refined_bboxes.size(); i++) {
      AINFO << refined_bboxes[i].rect;
    }

    _rectify->rectify(ros_image, hdmap_bboxes, refined_bboxes);
    _select->select(ros_image, hdmap_bboxes, refined_bboxes, green_bboxes);

    if (_enable_track) {
      if (!_init[option.camera_id]) {
        _kf[option.camera_id].statePost.at<float>(0) =
            green_bboxes[0].rect.x + cbox.rect.x -
                lights_ref[0]->region.projection_roi.x;
        _kf[option.camera_id].statePost.at<float>(1) =
            green_bboxes[0].rect.y + cbox.rect.y -
                lights_ref[0]->region.projection_roi.y;
        _kf[option.camera_id].statePre = _kf[option.camera_id].statePost.clone();

        _init[option.camera_id] = true;
      } else {
        cv::Mat measure(2, 1, CV_32F);
        measure.at<float>(0) =
            green_bboxes[0].rect.x + cbox.rect.x -
                lights_ref[0]->region.projection_roi.x;
        measure.at<float>(1) =
            green_bboxes[0].rect.y + cbox.rect.y -
                lights_ref[0]->region.projection_roi.y;
        _kf[option.camera_id].correct(measure);
        AINFO << "error:" << _kf[option.camera_id].statePost;
      }
    }
  } else {
    for (int h = 0; h < hdmap_bboxes.size(); h++) {
      BoundBox_t hbox = hdmap_bboxes[h];
      hbox.isValid = false;
      hbox.selected = false;
      green_bboxes.push_back(hbox);
    }
    _init[option.camera_id] = false;
  }

  for (int i = 0; i < lights_ref.size(); ++i) {
    if (!green_bboxes[i].isValid || !green_bboxes[i].selected) {
      XLOG(WARN) << "No detection box ,using project box";
    }
    cv::Rect region = green_bboxes[i].rect;
    region.x += cbox.rect.x;
    region.y += cbox.rect.y;

    lights_ref[i]->region.rectified_roi = region;
    if (!green_bboxes[i].isValid || !green_bboxes[i].selected) {
      XLOG(WARN) << "No detection box ,using project box";
      lights_ref[i]->region.is_detected = false;
    } else {
      lights_ref[i]->region.is_detected = true;
    }
    //std::vector<cv::Rect> temp;
    //temp.push_back(region);
    //lights_ref[i]->region.rectified_rois.push_back(temp);
    //lights_ref[i]->region.debug_roi.push_back(cbox.rect);
    AINFO << region;
  }
  return true;
}

std::string DenseBoxRectify::name() const {
  return "DenseBoxRectify";
}

REGISTER_RECTIFIER(DenseBoxRectify);

}
}
}
