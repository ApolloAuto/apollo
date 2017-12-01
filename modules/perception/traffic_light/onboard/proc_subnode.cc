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
#include "modules/perception/traffic_light/onboard/proc_subnode.h"
#include <std_msgs/String.h>
#include "ctime"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/lib/base/timer.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/traffic_light/base/tl_shared_data.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/recognizer/unity_recognize.h"
#include "modules/perception/traffic_light/rectify/cropbox.h"
#include "modules/perception/traffic_light/rectify/unity_rectify.h"
#include "modules/perception/traffic_light/reviser/color_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {

DEFINE_string(traffic_light_rectifier, "",
              "the rectifier enabled for traffic_light");
DEFINE_string(traffic_light_recognizer, "",
              "the recognizer enabled for traffic_light");
DEFINE_string(traffic_light_reviser, "",
              "the reviser enabled for traffic_light");

std::map<int, int> TLProcSubnode::_s_camera_ts_last_3_digits = {
    {static_cast<int>(CameraId::LONG_FOCUS), 222},
    {static_cast<int>(CameraId::SHORT_FOCUS), 111},
};
TLProcSubnode::~TLProcSubnode() {
  preprocessing_data_ = nullptr;
  proc_data_ = nullptr;
}

bool TLProcSubnode::InitInternal() {
  RegisterFactoryUnityRectify();
  RegisterFactoryUnityRecognize();
  RegisterFactoryColorReviser();

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
  if (!model_config->GetValue("image_border", &image_border_)) {
    AERROR << "TLProcSubnode Failed to find Conf: "
           << "image_border.";
    return false;
  }
  if (!model_config->GetValue("valid_ts_interval", &valid_ts_interval_)) {
    AERROR << "TLProcSubnode Failed to find Conf: "
           << "valid_ts_interval.";
    return false;
  }
  AINFO << "TLProcSubnode init successfully. ";
  return true;
}

bool TLProcSubnode::ProcEvent(const Event &event) {
  const double proc_subnode_handle_event_start_ts = TimeUtil::GetCurrentTime();
  PERF_FUNCTION();
  // get up-stream data
  const double timestamp = event.timestamp;
  const std::string device_id = event.reserve;

  AINFO << "Detect Start ts:" << GLOG_TIMESTAMP(timestamp);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id, &key)) {
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

  if (TimeUtil::GetCurrentTime() - event.local_timestamp > valid_ts_interval_) {
    AERROR << "TLProcSubnode failed to process image"
           << "Because images are too old"
           << ",current time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime())
           << ", event time: " << GLOG_TIMESTAMP(event.local_timestamp);
    return false;
  }

  // verify image_lights from cameras
  RectifyOption rectify_option;
  if (!VerifyImageLights(*image_lights, &rectify_option.camera_id)) {
    AERROR << "TLProcSubnode invalid image_lights ";
    return false;
  }

  if (!image_lights->image->GenerateMat()) {
    AERROR << "TLProcSubnode failed to generate mat";
    return false;
  }
  // using rectifier to rectify the region.
  const double before_rectify_ts = TimeUtil::GetCurrentTime();
  if (!rectifier_->Rectify(*(image_lights->image), rectify_option,
                           (image_lights->lights).get())) {
    AERROR << "TLProcSubnode failed to rectify the regions "
           << "ts:" << GLOG_TIMESTAMP(timestamp)
           << " Image:" << *(image_lights->image);
    return false;
  }
  const double detection_latency =
      TimeUtil::GetCurrentTime() - before_rectify_ts;

  // update image_border
  MutexLock lock(&mutex_);
  // int cam_id = static_cast<int>(image_lights->camera_id);
  ComputeImageBorder(*image_lights,
                     &image_border_size[image_lights->camera_id]);
  AINFO << "TLProcSubnode update image_border size: "
        << image_border_size[image_lights->camera_id]
        << " ts: " << GLOG_TIMESTAMP(timestamp)
        << " CameraId: " << image_lights->camera_id;
  image_lights->offset = image_border_size[image_lights->camera_id];

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
  if (!reviser_->Revise(ReviseOption(event.timestamp),
                        image_lights->lights.get())) {
    AERROR << "TLReviserSubnode revise data failed. "
           << "sub_event:" << event.to_string();
    return false;
  }
  const double revise_latency = TimeUtil::GetCurrentTime() - before_revise_ts;
  PublishMessage(image_lights);
  AINFO << "TLProcSubnode process traffic_light, "
        << " msg_ts: " << GLOG_TIMESTAMP(timestamp)
        << " from device_id: " << device_id << " get "
        << image_lights->lights->size() << " lights."
        << " detection_latency: " << detection_latency * 1000 << " ms."
        << " recognization_latency: " << recognization_latency * 1000 << " ms."
        << " revise_latency: " << revise_latency * 1000 << " ms."
        << " TLProcSubnode::handle_event latency: "
        << (TimeUtil::GetCurrentTime() - proc_subnode_handle_event_start_ts) *
               1000
        << " ms."
        << " enter_proc_latency: " << enter_proc_latency * 1000 << " ms."
        << " preprocess_latency: "
        << (image_lights->preprocess_send_timestamp -
            image_lights->preprocess_receive_timestamp) *
               1000
        << " ms.";
  // }
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
  AINFO << "Init shared data successfully, "
        << "preprocessing_data: " << preprocessing_data_->name();
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
  reviser_.reset(
      BaseReviserRegisterer::GetInstanceByName(FLAGS_traffic_light_reviser));
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

bool TLProcSubnode::VerifyImageLights(const ImageLights &image_lights,
                                      CameraId *selection) const {
  if (!image_lights.image || !image_lights.image->contain_image()) {
    AERROR << "TLProcSubnode image_lights has no image, "
           << "verify_image_lights failed.";
    return false;
  }

  const int cam_id = static_cast<int>(image_lights.camera_id);
  if (cam_id < 0 || cam_id >= kCountCameraId) {
    AERROR << "TLProcSubnode image_lights unknown camera id: " << cam_id
           << " verify_image_lights failed.";
    return false;
  }
  for (LightPtr light : *(image_lights.lights)) {
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
  if (camera_id < 0 || camera_id >= kCountCameraId) {
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

void TLProcSubnode::ComputeRectsOffset(const cv::Rect &rect1,
                                       const cv::Rect &rect2, int *offset) {
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

bool TLProcSubnode::PublishMessage(
    const std::shared_ptr<ImageLights> &image_lights) const {
  Timer timer;
  timer.Start();
  const auto &lights = image_lights->lights;
  cv::Mat img = image_lights->image->mat();
  apollo::perception::TrafficLightDetection result;
  apollo::common::Header *header = result.mutable_header();
  header->set_timestamp_sec(ros::Time::now().toSec());
  uint64_t timestamp = TimestampDouble2Int64(image_lights->image->ts());
  timestamp += TLProcSubnode::_s_camera_ts_last_3_digits[static_cast<int>(
      image_lights->image->device_id())];

  header->set_camera_timestamp(timestamp);
  // add traffic light result
  for (size_t i = 0; i < lights->size(); i++) {
    apollo::perception::TrafficLight *light_result = result.add_traffic_light();
    light_result->set_id(lights->at(i)->info.id().id());
    light_result->set_confidence(lights->at(i)->status.confidence);
    light_result->set_color(lights->at(i)->status.color);
    cv::Rect rect = lights->at(i)->region.rectified_roi;
    cv::Scalar color;
    switch (lights->at(i)->status.color) {
      case BLACK:
        color = cv::Scalar(0, 0, 0);
        break;
      case GREEN:
        color = cv::Scalar(0, 255, 0);
        break;
      case RED:
        color = cv::Scalar(0, 0, 255);
        break;
      case YELLOW:
        color = cv::Scalar(0, 255, 255);
        break;
      default:
        color = cv::Scalar(0, 76, 153);
    }
    cv::rectangle(img, rect, color, 2);
  }

  // set contain_lights
  result.set_contain_lights(image_lights->num_signals > 0);

  // add traffic light debug info
  apollo::perception::TrafficLightDebug *light_debug =
      result.mutable_traffic_light_debug();

  // set signal number
  AINFO << "TLOutputSubnode num_signals: " << image_lights->num_signals
        << ", camera_id: " << kCameraIdToStr.at(image_lights->camera_id)
        << ", is_pose_valid: " << image_lights->is_pose_valid
        << ", ts: " << GLOG_TIMESTAMP(image_lights->timestamp);
  light_debug->set_signal_num(image_lights->num_signals);

  // Crop ROI
  if (lights->size() > 0 && lights->at(0)->region.debug_roi.size() > 0) {
    auto &crop_roi = lights->at(0)->region.debug_roi[0];
    auto tl_cropbox = light_debug->mutable_cropbox();
    tl_cropbox->set_x(crop_roi.x);
    tl_cropbox->set_y(crop_roi.y);
    tl_cropbox->set_width(crop_roi.width);
    tl_cropbox->set_height(crop_roi.height);
    cv::rectangle(img, crop_roi, cv::Scalar(0, 255, 255), 2);
  }

  // Rectified ROI
  for (size_t i = 0; i < lights->size(); ++i) {
    auto &rectified_roi = lights->at(i)->region.rectified_roi;
    auto tl_rectified_box = light_debug->add_box();
    tl_rectified_box->set_x(rectified_roi.x);
    tl_rectified_box->set_y(rectified_roi.y);
    tl_rectified_box->set_width(rectified_roi.width);
    tl_rectified_box->set_height(rectified_roi.height);
    tl_rectified_box->set_color(lights->at(i)->status.color);
    tl_rectified_box->set_selected(true);
  }

  // Projection ROI
  for (size_t i = 0; i < lights->size(); ++i) {
    auto &projection_roi = lights->at(i)->region.projection_roi;
    auto tl_projection_box = light_debug->add_box();
    tl_projection_box->set_x(projection_roi.x);
    tl_projection_box->set_y(projection_roi.y);
    tl_projection_box->set_width(projection_roi.width);
    tl_projection_box->set_height(projection_roi.height);
  }

  // debug ROI (candidate detection boxes)
  if (lights->size() > 0 && lights->at(0)->region.debug_roi.size() > 0) {
    for (size_t i = 1; i < lights->at(0)->region.debug_roi.size(); ++i) {
      auto &debug_roi = lights->at(0)->region.debug_roi[i];
      auto tl_debug_box = light_debug->add_box();
      tl_debug_box->set_x(debug_roi.x);
      tl_debug_box->set_y(debug_roi.y);
      tl_debug_box->set_width(debug_roi.width);
      tl_debug_box->set_height(debug_roi.height);
    }
  }

  light_debug->set_ts_diff_pos(image_lights->diff_image_pose_ts);
  light_debug->set_ts_diff_sys(image_lights->diff_image_sys_ts);
  light_debug->set_valid_pos(image_lights->is_pose_valid);
  light_debug->set_project_error(image_lights->offset);

  if (lights->size() > 0) {
    double distance = stopline_distance(image_lights->pose.pose(),
                                        lights->at(0)->info.stop_line());
    light_debug->set_distance_to_stop_line(distance);
  }
  char filename[100];
  snprintf(filename, 200, "img/%s_%lf.jpg",
           image_lights->image->device_id_str().c_str(),
           image_lights->image->ts());
  cv::imwrite(filename, img);
  common::adapter::AdapterManager::PublishTrafficLightDetection(result);
  auto process_time =
      TimeUtil::GetCurrentTime() - image_lights->preprocess_receive_timestamp;
  AINFO << "Publish message "
        << " ts:" << GLOG_TIMESTAMP(image_lights->timestamp)
        << " device:" << image_lights->image->device_id_str() << " consuming "
        << process_time * 1000 << " ms."
        << " number of lights:" << lights->size()
        << " lights:" << result.ShortDebugString();

  timer.End("TLProcSubnode::Publish message");
  return true;
}
StatusCode TLProcSubnode::ProcEvents() {
  Event event;
  const EventMeta &event_meta = sub_meta_events_[0];
  if (!event_manager_->Subscribe(event_meta.event_id, &event)) {
    AERROR << "Failed to subscribe event: " << event_meta.event_id;
    return FAIL;
  }
  if (!ProcEvent(event)) {
    AERROR << "TLProcSubnode failed to handle event. "
           << "event:" << event.to_string();
    return FAIL;
  }
  return SUCC;
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
