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
#include "modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h"

#include "image_transport/image_transport.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/utils.h"
#include "modules/perception/traffic_light/projection/projection.h"
#include "modules/perception/traffic_light/recognizer/unity_recognize.h"
#include "modules/perception/traffic_light/rectify/unity_rectify.h"
#include "modules/perception/traffic_light/reviser/color_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {

using common::adapter::AdapterManager;

bool TLPreprocessorSubnode::InitInternal() {
  RegisterFactoryBoundaryProjection();
  if (!InitSharedData()) {
    AERROR << "TLPreprocessorSubnode init failed. Shared Data init failed.";
    return false;
  }

  ConfigManager *config_manager = ConfigManager::instance();
  std::string model_name("TLPreprocessorSubnode");
  const ModelConfig *model_config = config_manager->GetModelConfig(model_name);
  if (model_config == nullptr) {
    AERROR << "TLPreprocessorSubnode not found model: " << model_name;
    return false;
  }
  float max_process_image_fps_;
  if (!model_config->GetValue("max_process_image_fps",
                              &max_process_image_fps_)) {
    AERROR << "TLPreprocessorSubnode Failed to find Conf: "
           << "max_process_image_fps.";
    return false;
  }
  proc_interval_seconds_ = 1.0f / max_process_image_fps_;

  if (!model_config->GetValue("query_tf_inverval_seconds",
                              &query_tf_inverval_seconds_)) {
    AERROR << "TLPreprocessorSubnode Failed to find Conf: "
           << "query_tf_inverval_seconds.";
    return false;
  }
  // init preprocessor
  if (!InitPreprocessor()) {
    AERROR << "TLPreprocessorSubnode init failed.";
    return false;
  }

  // init hd_map
  if (!InitHdmap()) {
    AERROR << "TLPreprocessorSubnode Failed to init hdmap";
    return false;
  }

  CHECK(AdapterManager::GetImageLong())
      << "TLPreprocessorSubnode init failed.ImageLong is not initialized.";
  AdapterManager::AddImageLongCallback(
      &TLPreprocessorSubnode::SubLongFocusCamera, this);
  CHECK(AdapterManager::GetImageShort())
      << "TLPreprocessorSubnode init failed.ImageShort is not initialized.";
  AdapterManager::AddImageShortCallback(
      &TLPreprocessorSubnode::SubShortFocusCamera, this);
  return true;
}

bool TLPreprocessorSubnode::InitSharedData() {
  CHECK_NOTNULL(shared_data_manager_);

  const std::string preprocessing_data_name("TLPreprocessingData");
  preprocessing_data_ = dynamic_cast<TLPreprocessingData *>(
      shared_data_manager_->GetSharedData(preprocessing_data_name));
  if (preprocessing_data_ == nullptr) {
    AERROR << "TLPreprocessorSubnode failed to get shared data instance "
           << preprocessing_data_name;
    return false;
  }
  AINFO << "TLPreprocessorSubnode init shared data. name:"
        << preprocessing_data_->name();
  return true;
}

bool TLPreprocessorSubnode::InitPreprocessor() {
  if (!preprocessor_.Init()) {
    AERROR << "TLPreprocessorSubnode init preprocessor failed";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::InitHdmap() {
  hd_map_ = HDMapInput::instance();
  if (hd_map_ == nullptr) {
    AERROR << "TLPreprocessorSubnode get hdmap failed.";
    return false;
  }
  if (!hd_map_->Init()) {
    AERROR << "TLPreprocessorSubnode init hd-map failed.";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::AddDataAndPublishEvent(
    const std::shared_ptr<ImageLights> &data, const CameraId &camera_id,
    double timestamp) {
  // add data down-stream
  std::string device_str = kCameraIdToStr.at(camera_id);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_str, &key)) {
    AERROR << "TLPreprocessorSubnode gen share data key failed. ts:"
           << GLOG_TIMESTAMP(timestamp);
    return false;
  }

  if (!preprocessing_data_->Add(key, data)) {
    AERROR << "TLPreprocessorSubnode push data into shared_data failed.";
    data->image.reset();
    return false;
  }

  // pub events
  for (size_t i = 0; i < this->pub_meta_events_.size(); ++i) {
    const EventMeta &event_meta = this->pub_meta_events_[i];
    Event event;
    event.event_id = event_meta.event_id;
    event.reserve = device_str;
    event.timestamp = timestamp;
    this->event_manager_->Publish(event);
  }
  return true;
}

void TLPreprocessorSubnode::SubLongFocusCamera(const sensor_msgs::Image &msg) {
  AdapterManager::Observe();
  SubCameraImage(AdapterManager::GetImageLong()->GetLatestObservedPtr(),
                 LONG_FOCUS);
  PERF_FUNCTION("SubLongFocusCamera");
}

void TLPreprocessorSubnode::SubShortFocusCamera(const sensor_msgs::Image &msg) {
  AdapterManager::Observe();
  SubCameraImage(AdapterManager::GetImageShort()->GetLatestObservedPtr(),
                 SHORT_FOCUS);
  PERF_FUNCTION("SubShortFocusCamera");
}

void TLPreprocessorSubnode::SubCameraImage(
    boost::shared_ptr<const sensor_msgs::Image> msg, CameraId camera_id) {
  const double sub_camera_image_start_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<Image> image(new Image);
  cv::Mat cv_mat;
  double timestamp = msg->header.stamp.toSec();
  image->Init(timestamp, camera_id, msg);
  if (FLAGS_output_raw_img) {
    // user should create folders
    image->GenerateMat();
    char filename[100];
    snprintf(filename, sizeof(filename), "%s/%lf.jpg",
             image->camera_id_str().c_str(), timestamp);
    cv::imwrite(filename, image->mat());
  }
  AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << kCameraIdToStr.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec());

  // which camera should be used?  called in low frequence
  CameraSelection(timestamp);

  AINFO << "sub_camera_image_start_ts: "
        << GLOG_TIMESTAMP(sub_camera_image_start_ts)
        << " , last_proc_image_ts_: " << GLOG_TIMESTAMP(last_proc_image_ts_)
        << " , diff: "
        << GLOG_TIMESTAMP(sub_camera_image_start_ts - last_proc_image_ts_);
  if (last_proc_image_ts_ > 0.0 &&
      sub_camera_image_start_ts - last_proc_image_ts_ <
          proc_interval_seconds_) {
    AINFO << "skip current image, img_ts: " << GLOG_TIMESTAMP(timestamp)
          << " ,because proc_interval_seconds_: "
          << GLOG_TIMESTAMP(proc_interval_seconds_);
    return;
  }

  // sync image and publish data
  const double before_sync_image_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<ImageLights> image_lights(new ImageLights);
  bool should_pub = false;
  if (!preprocessor_.SyncImage(image, &image_lights, &should_pub)) {
    AINFO << "sync image failed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  } else {
    AINFO << "sync image succeed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  }
  const double sync_image_latency =
      TimeUtil::GetCurrentTime() - before_sync_image_ts;

  // Monitor image time and system time difference
  int max_cached_lights_size = preprocessor_.max_cached_lights_size();
  // tf frequency is 100Hz, 0.01 sec per frame，
  // cache frame num: max_cached_image_lights_array_size * 0.005 tf info
  const float tf_interval = 0.01;
  double image_sys_ts_diff_threshold = max_cached_lights_size * tf_interval;
  if (fabs(image_lights->diff_image_sys_ts) > image_sys_ts_diff_threshold) {
    std::string debug_string = "";
    debug_string += ("diff_image_sys_ts:" +
                     std::to_string(image_lights->diff_image_sys_ts));
    debug_string += (",camera_id:" + kCameraIdToStr.at(camera_id));
    debug_string += (",camera_ts:" + std::to_string(timestamp));

    AWARN << "image_ts - system_ts(in seconds): "
          << std::to_string(image_lights->diff_image_sys_ts)
          << ". Check if image timestamp drifts."
          << ", camera_id: " + kCameraIdToStr.at(camera_id)
          << ", debug_string: " << debug_string;
  }

  if (!should_pub) {
    AINFO << "TLPreprocessorSubnode not publish image, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
  }

  // verify lights projection based on image time
  if (!VerifyLightsProjection(image_lights)) {
    AINFO << "verify_lights_projection on image failed, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
  }

  // record current frame timestamp
  last_proc_image_ts_ = sub_camera_image_start_ts;

  image_lights->preprocess_receive_timestamp = sub_camera_image_start_ts;
  image_lights->preprocess_send_timestamp = TimeUtil::GetCurrentTime();
  if (AddDataAndPublishEvent(image_lights, camera_id, image->ts())) {
    preprocessor_.set_last_pub_camera_id(camera_id);
    AINFO << "TLPreprocessorSubnode::sub_camera_image msg_time: "
          << GLOG_TIMESTAMP(image->ts())
          << " sync_image_latency: " << sync_image_latency * 1000 << " ms."
          << " sub_camera_image_latency: "
          << (TimeUtil::GetCurrentTime() - sub_camera_image_start_ts) * 1000
          << " ms."
          << " camera_id: " << kCameraIdToStr.at(camera_id);
    AINFO << " number of lights: " << image_lights->lights->size();
  }
}

bool TLPreprocessorSubnode::GetSignals(double ts, CarPose *pose,
                                       std::vector<Signal> *signals) {
  // get pose
  if (!GetCarPose(ts, pose)) {
    AERROR << "camera_selection failed to get car pose, ts:"
           << GLOG_TIMESTAMP(ts);
    return false;
  }
  AINFO << "camera_selection get position\n " << std::setprecision(12)
        << pose->pose();

  // get signals
  if (!hd_map_->GetSignals(pose->pose(), signals)) {
    if (ts - last_signals_ts_ < valid_hdmap_interval_) {
      *signals = last_signals_;
      AWARN << "camera_selection failed to get signals info. "
            << "Now use last info. ts:" << GLOG_TIMESTAMP(ts)
            << " pose:" << *pose;
    } else {
      AERROR << "camera_selection failed to get signals info. "
             << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << *pose;
      return false;
    }
  } else {
    last_signals_ = *signals;
    last_signals_ts_ = ts;
  }
  return true;
}
bool TLPreprocessorSubnode::GetCarPose(const double ts, CarPose *pose) {
  Eigen::Matrix4d pose_matrix;

  if (!GetVelodyneTrans(ts, &pose_matrix)) {
    AERROR << "TLPreprocessorSubnode failed to query pose ts:"
           << GLOG_TIMESTAMP(ts);
    return false;
  }
  pose->set_pose(pose_matrix);
  return true;
}
bool TLPreprocessorSubnode::VerifyLightsProjection(
    ImageLightsPtr image_lights) {
  std::vector<Signal> signals;
  CarPose pose;
  if (!GetSignals(image_lights->timestamp, &pose, &signals)) {
    return false;
  }

  // TODO(ghdawn): no need to init lights before this line
  image_lights->num_signals = signals.size();
  image_lights->lights.reset(new LightPtrs);
  image_lights->lights_outside_image.reset(new LightPtrs);
  if (!preprocessor_.ProjectLights(pose, signals, image_lights->camera_id,
                                   image_lights->lights.get(),
                                   image_lights->lights_outside_image.get())) {
    AINFO << "preprocessor_.select_camera_by_lights_projection failed";
    return false;
  }

  return true;
}
void TLPreprocessorSubnode::CameraSelection(double ts) {
  const double current_ts = TimeUtil::GetCurrentTime();
  AINFO << "current_ts: " << GLOG_TIMESTAMP(current_ts)
        << " , last_query_tf_ts: " << GLOG_TIMESTAMP(last_query_tf_ts_)
        << " , diff: " << GLOG_TIMESTAMP(current_ts - last_query_tf_ts_);
  if (last_query_tf_ts_ > 0.0 &&
      current_ts - last_query_tf_ts_ < query_tf_inverval_seconds_) {
    AINFO << "skip current tf msg, img_ts: " << GLOG_TIMESTAMP(ts);
    return;
  }

  CarPose pose;
  std::vector<Signal> signals;
  if (!GetSignals(ts, &pose, &signals)) {
    return;
  }
  if (!preprocessor_.CacheLightsProjections(pose, signals, ts)) {
    AERROR << "add_cached_lights_projections failed, ts: "
           << GLOG_TIMESTAMP(ts);
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: "
          << GLOG_TIMESTAMP(ts);
  }
  last_query_tf_ts_ = current_ts;
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
