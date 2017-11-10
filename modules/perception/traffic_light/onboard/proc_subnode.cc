// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/08
// @file: proc_subnode.cpp
// @brief: 
// 
#include "modules/perception/traffic_light/onboard/proc_subnode.h"
#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/rectify/cropbox.h"
#include "ctime"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/traffic_light/onboard/proc_data.h"
#include "modules/perception/traffic_light/onboard/preprocessor_data.h"
#include "modules/perception/traffic_light/onboard/preprocessor_subnode.h"

namespace apollo {
namespace perception {
namespace traffic_light {

DEFINE_string(traffic_light_rectifier,
              "",
              "the rectifier enabled for traffic_light");
DEFINE_string(traffic_light_recognizer,
              "",
              "the recognizer enabled for traffic_light");
DEFINE_string(traffic_light_reviser,
              "", "the reviser enabled for traffic_light");

DEFINE_double(valid_ts_interval,
              100,
              "the difference between event ts and now ts must be less than me.Unit:second ");

TLProcSubnode::~TLProcSubnode() {
  preprocessing_data_ = nullptr;
  proc_data_ = nullptr;
}

bool TLProcSubnode::InitInternal() {

  if (!InitSharedData()) {
    AERROR << "TLProcSubnode init shared data failed.";
    return false;
  }
  if (!InitRectifier()) {
    AERROR << "TLProcSubnode init rectifier failed.";
    return false;
  }
  if (!InitRecognizer()) {
    AERROR << "TLProcSubnode init recognizer failed.";
    return false;
  }
  if (!InitReviser()) {
    AERROR << "TLProcSubnode init reviser failed.";
    return false;
  }

  // init image_border
  ConfigManager *config_manager = ConfigManager::instance();
  std::string model_name("TLProcSubnode");
  const ModelConfig *model_config(nullptr);
  if (!config_manager->GetModelConfig(model_name, &model_config)) {
    AERROR << "TLProcSubnode not found model: " << model_name;
    return false;
  }
  if (!model_config->GetValue("image_border",
                              &image_border_)) {
    AERROR << "TLProcSubnode Failed to find Conf: "
           << "image_border.";
    return false;
  }
  int crop_method = 0;
  switch (crop_method) {
    default:
    case 0: {
      float crop_scale = 0;
      float crop_min_size = 0;
      crop_.reset(new CropBox(crop_scale, crop_min_size));
    }
      break;
    case 1:crop_.reset(new CropBoxWholeImage());
      break;
  }

  AINFO << "TLProcSubnode init successfully. ";
  return true;
}

bool TLProcSubnode::HandleEvent(const Event &sub_event,
                                Event *pub_event) {
  const double proc_subnode_handle_event_start_ts = TimeUtil::GetCurrentTime();
  PERF_FUNCTION();
  // get up-stream data
  const double timestamp = sub_event.timestamp;
  const std::string device_id = sub_event.reserve;
  pub_event->local_timestamp = TimeUtil::GetCurrentTime();

  AINFO << "Detect Start ts:" << GLOG_TIMESTAMP(timestamp);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id,
                                           &key)) {
    AERROR << "TLProcSubnode produce_shared_data_key failed."
           << " ts:" << timestamp << " device_id:" << device_id;
    return false;
  }

  SharedDataPtr<ImageLights> image_lights;
  if (!preprocessing_data_->Get(key, &image_lights)) {
    AERROR << "TLProcSubnode failed to get shared data,"
           << " name:" << preprocessing_data_->name()
           << ", time: " << GLOG_TIMESTAMP(timestamp);
    return false;
  }
  AINFO << "TLProcSubnode get shared data ok,ts: " << GLOG_TIMESTAMP(timestamp);

  // preprocess send a msg -> proc receive a msg
  double enter_proc_latency = (proc_subnode_handle_event_start_ts -
      image_lights->preprocess_send_timestamp);

  if (TimeUtil::GetCurrentTime() - sub_event.local_timestamp > FLAGS_valid_ts_interval) {
    AERROR << "TLProcSubnode failed to process image"
           << "Because images are too old"
           << ",current time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime())
           << ", event time: " << GLOG_TIMESTAMP(sub_event.local_timestamp);
    return false;
  }

  // verify image_lights from cameras
  RectifyOption rectify_option;
  if (!VerifyImageLights(*image_lights, &rectify_option.camera_id)) {
    AERROR << "TLProcSubnode invalid image_lights ";
    return false;
  }

  //cv::Rect cbox;
  //_crop->get_crop_box(image_lights->image->size(), *(image_lights->lights), &cbox);
  if (!image_lights->image->GenerateMat()) {
    AERROR << "TLProcSubnode failed to generate mat";
    return false;
  }
  // using rectifier to rectify the region.
  const double before_rectify_ts = TimeUtil::GetCurrentTime();
  if (!rectifier_->Rectify(*(image_lights->image), rectify_option,
                           (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to rectify the regions "
           << "ts:" << GLOG_TIMESTAMP(timestamp) << " Image:" << *(image_lights->image);
    return false;
  }
  const double detection_latency = TimeUtil::GetCurrentTime() - before_rectify_ts;

  // update image_border
  const double before_update_image_border_ts = TimeUtil::GetCurrentTime();
  MutexLock lock(&mutex_);
  //int cam_id = static_cast<int>(image_lights->camera_id);
  ComputeImageBorder(*image_lights,
                     &TLPreprocessorSubnode::_s_image_borders[image_lights->camera_id]);
  AINFO << "TLProcSubnode update image_border size: "
        << TLPreprocessorSubnode::_s_image_borders[image_lights->camera_id]
        << " ts: " << GLOG_TIMESTAMP(timestamp)
        << " CameraId: " << image_lights->camera_id;
  image_lights->offset = TLPreprocessorSubnode::_s_image_borders[image_lights->camera_id];
  const double update_image_border_latency =
      TimeUtil::GetCurrentTime() - before_update_image_border_ts;

  // recognize_status
  const double before_recognization_ts = TimeUtil::GetCurrentTime();
  if (!recognizer_->RecognizeStatus(*(image_lights->image), RecognizeOption(),
                                    (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to recognize lights,"
           << " ts:" << GLOG_TIMESTAMP(timestamp)
           << " image:" << image_lights->image;
    return false;
  }
  const double recognization_latency =
      TimeUtil::GetCurrentTime() - before_recognization_ts;

  // revise status
  const double before_revise_ts = TimeUtil::GetCurrentTime();
  if (!reviser_->Revise(ReviseOption(sub_event.timestamp), image_lights->lights.get())) {
    AERROR << "TLReviserSubnode revise data failed. "
           << "sub_event:" << sub_event.to_string();
    return false;
  }
  const double revise_latency = TimeUtil::GetCurrentTime() - before_revise_ts;

  AINFO << "TLProcSubnode process traffic_light, "
        << " msg_ts: " << GLOG_TIMESTAMP(timestamp)
        << " from device_id: " << device_id
        << " get " << image_lights->lights->size() << " lights."
        << " detection_latency: " << detection_latency * 1000 << " ms."
        << " recognization_latency: " << recognization_latency * 1000 << " ms."
        << " revise_latency: " << revise_latency * 1000 << " ms."
        << " TLProcSubnode::handle_event latency: "
        << (TimeUtil::GetCurrentTime() -
            proc_subnode_handle_event_start_ts) * 1000 << " ms."
        << " enter_proc_latency: " << enter_proc_latency * 1000 << " ms."
        << " preprocess_latency: " << (image_lights->preprocess_send_timestamp -
      image_lights->preprocess_receive_timestamp) * 1000
        << " ms.";
  // }

  // add to down-stream data
  if (!proc_data_->Add(key, image_lights)) {
    AERROR << "TLProcSubnode failed to add data down-stream, "
           << " key:" << key;
    return false;
  }

  // set pub_event
  pub_event->timestamp = timestamp;
  pub_event->reserve = device_id;

  return true;
}

bool TLProcSubnode::InitSharedData() {
  CHECK_NOTNULL(shared_data_manager_);

  const std::string preprocessing_data_name("TLPreprocessingData");
  preprocessing_data_ = dynamic_cast<TLPreprocessingData *>(
      shared_data_manager_->GetSharedData(preprocessing_data_name));
  if (preprocessing_data_ == nullptr) {
    AERROR << "TLProcSubnode failed to get shared data instance: "
           << preprocessing_data_name;
    return false;
  }

  const std::string proc_data_name("TLProcData");
  proc_data_ = dynamic_cast<TLProcData *>(
      shared_data_manager_->GetSharedData(proc_data_name));
  if (proc_data_ == nullptr) {
    AERROR << "Failed to get shared data instance: "
           << proc_data_name;
    return false;
  }

  AINFO << "Init shared data successfully, "
        << "preprocessing_data: " << preprocessing_data_->name()
        << "proc_data:" << proc_data_->name();
  return true;
}

bool TLProcSubnode::InitRectifier() {

  rectifier_.reset(BaseRectifierRegisterer::GetInstanceByName(
      FLAGS_traffic_light_rectifier));
  if (!rectifier_) {
    AERROR << "TLProcSubnode new rectifier failed. rectifier name:"
           << FLAGS_traffic_light_rectifier << " failed.";
    return false;
  }
  if (!rectifier_->Init()) {
    AERROR << "TLProcSubnode init rectifier failed. rectifier name:"
           << FLAGS_traffic_light_rectifier << " failed.";
    return false;
  }
  return true;
}

bool TLProcSubnode::InitRecognizer() {
  recognizer_.reset(BaseRecognizerRegisterer::GetInstanceByName(
      FLAGS_traffic_light_recognizer));
  if (!recognizer_) {
    AERROR << "TLProcSubnode new recognizer failed. name:"
           << FLAGS_traffic_light_recognizer;
    return false;
  }
  if (!recognizer_->Init()) {
    AERROR << "TLProcSubnode init recognizer failed.";
    return false;
  }
  return true;
}

bool TLProcSubnode::InitReviser() {
  reviser_.reset(BaseReviserRegisterer::GetInstanceByName(
      FLAGS_traffic_light_reviser));
  if (reviser_ == nullptr) {
    AERROR << "TLProcSubnode new reviser failed. name:"
           << FLAGS_traffic_light_reviser;
    return false;
  }
  if (!reviser_->Init()) {
    AERROR << "TLProcSubnode init reviser failed. name:"
           << FLAGS_traffic_light_reviser;
    return false;
  }
  return true;
}

double TLProcSubnode::GetMeanDistance(const double ts,
                                      const Eigen::Matrix4d &car_pose,
                                      const LightPtrs &lights) const {
  if (lights.empty()) {
    AWARN << "get_mean_distance failed. lights is empty, "
          << "while it should not be. ts:" << GLOG_TIMESTAMP(ts);
    return DBL_MAX;
  }

  double distance = 0.0;
  for (const LightPtr &light : lights) {
    auto light_distance = stopline_distance(car_pose, light->info.stop_line());
    if (light_distance < 0) {
      AWARN << "get_mean_distance failed. lights stop line data is illegal, "
            << "ts:" << GLOG_TIMESTAMP(ts);
      return DBL_MAX;
    }
    distance += light_distance;
  }
  return distance / lights.size();
}

bool TLProcSubnode::VerifyImageLights(
    const ImageLights &image_lights, CameraId *selection) const {
  if (!image_lights.image || !image_lights.image->contain_image()) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "verify_image_lights failed.";
    return false;
  }

  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  const int cam_id = static_cast<int>(image_lights.camera_id);
  if (cam_id < 0 || cam_id >= num_camera_ids) {
    AERROR << "TLProcSubnode image_lights unknown camera id, "
           << "verify_image_lights failed.";
    return false;
  }
  for (LightPtr light:*(image_lights.lights)) {
    if (!BoxIsValid(light->region.projection_roi, image_lights.image->size())) {
      ClearBox(light->region.projection_roi);
      continue;
    }
  }
  *selection = image_lights.camera_id;

  return true;
}

bool TLProcSubnode::ComputeImageBorder(const ImageLights &image_lights,
                                       int *image_border) {
  if (!image_lights.image) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "compute_image_border failed.";
    return false;
  }

  auto camera_id = static_cast<int>(image_lights.camera_id);
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  if (camera_id < 0 || camera_id >= num_camera_ids) {
    AERROR << "TLProcSubnode image_lights unknown camera selection, "
           << "compute_image_border failed, "
           << "camera_id: " << kCameraIdToStr.at(image_lights.camera_id);
    return false;
  }

  // check lights info
  if (image_lights.lights->empty()) {
    AINFO << "TLProcSubnode image_lights no lights info, "
          << "no need to update image border, reset image border size to 100";
    *image_border = 100;
    return true;
  }

  if (camera_id == static_cast<int>(CameraId::UNKNOWN) - 1) {
    AINFO << "TLProcSubnode no need to update image border, "
          << "camera_id: " << kCameraIdToStr.at(image_lights.camera_id);
    return true;
  }

  LightPtrs &lights_ref = *(image_lights.lights.get());
  int max_offset = -1;
  for (size_t i = 0; i < lights_ref.size(); ++i) {
    cv::Rect rectified_roi = lights_ref[i]->region.rectified_roi;
    cv::Rect projection_roi = lights_ref[i]->region.projection_roi;
    // 有多个灯，取最大偏移
    int offset = 0;
    ComputeRectsOffset(projection_roi, rectified_roi, &offset);
    max_offset = std::max(max_offset, offset);
  }
  if (max_offset != -1) {
    *image_border = max_offset;
  }

  return true;
}

void TLProcSubnode::ComputeRectsOffset(
    const cv::Rect &rect1,
    const cv::Rect &rect2,
    int *offset) {
  cv::Point center1(rect1.x + rect1.width / 2, rect1.y + rect1.height / 2);
  cv::Point center2(rect2.x + rect2.width / 2, rect2.y + rect2.height / 2);

  cv::Point pt1;
  cv::Point pt2;
  // 分四个象限, 记录横、纵方向最大偏移
  if (center2.y <= center1.y) {
    if (center2.x >= center1.x) {
      pt1 = cv::Point(rect1.x + rect1.width, rect1.y);
      pt2 = cv::Point(rect2.x + rect2.width, rect2.y);
    } else {
      pt1 = cv::Point(rect1.x, rect1.y);
      pt2 = cv::Point(rect2.x, rect2.y);
    }
  } else {
    if (center2.x >= center1.x) {
      pt1 = cv::Point(rect1.x + rect1.width, rect1.y + rect1.height);
      pt2 = cv::Point(rect2.x + rect2.width, rect2.y + rect2.height);
    } else {
      pt1 = cv::Point(rect1.x, rect1.y + rect1.height);
      pt2 = cv::Point(rect2.x, rect2.y + rect2.height);
    }
  }

  *offset = std::max(abs(pt1.x - pt2.x), abs(pt1.y - pt2.y));
}

} // namespace traffic_light
} // namespace perception
} // namespace apollo
