// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/12 16:55:49
// @file: preprocessor_subnode.cpp
// @brief: preprocessor_subnode definition.
//
#include "modules/perception/traffic_light/onboard/preprocessor_subnode.h"

#include <image_transport/image_transport.h>
#include "modules/perception/traffic_light/recognizer/unity_recognize.h"
#include "modules/perception/traffic_light/rectify/unity_rectify.h"
#include "modules/perception/traffic_light/reviser/color_decision.h"
#include "modules/perception/traffic_light/projection/projection.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/onboard/transform_input.h"
#include "modules/perception/traffic_light/base/utils.h"

DEFINE_double(min_valid_ts_in_seconds, 0.0,
"min valid timestamp, if ts < min_valid_ts_in_seconds image will be skipped.");
DEFINE_double(max_valid_ts_in_seconds, FLT_MAX,
              "max valid timestamp, if ts > max_valid_ts_in_seconds image will be skipped.");

namespace apollo {
namespace perception {
namespace traffic_light {

std::map<int, int> TLPreprocessorSubnode::_s_camera_ts_last_3_digits = {
    {static_cast<int>(CameraId::LONG_FOCUS), 222},
    {static_cast<int>(CameraId::SHORT_FOCUS), 111},
};

std::map<int, std::string> TLPreprocessorSubnode::_s_camera_names = {
    {static_cast<int>(CameraId::LONG_FOCUS), "long_focus_camera"},
    {static_cast<int>(CameraId::SHORT_FOCUS), "short_focus_camera"},
};

std::map<int, int> TLPreprocessorSubnode::_s_image_borders = {
    {static_cast<int>(CameraId::LONG_FOCUS), 100},
    {static_cast<int>(CameraId::SHORT_FOCUS), 100},
};

bool TLPreprocessorSubnode::InitInternal() {
  RegisterFactorySingleBoundaryBasedProjection();
  if (!init_shared_data()) {
    AERROR << "TLPreprocessorSubnode init failed. Shared Data init failed.";
    return false;
  }

  ConfigManager *config_manager = ConfigManager::instance();
  std::string model_name("TLPreprocessorSubnode");
  const ModelConfig *model_config(nullptr);
  if (!config_manager->GetModelConfig(model_name, &model_config)) {
    AERROR << "TLPreprocessorSubnode not found model: " << model_name;
    return false;
  }
  if (!model_config->GetValue("max_process_image_fps",
                              &_max_process_image_fps)) {
    AERROR << "TLPreprocessorSubnode Failed to find Conf: "
           << "max_process_image_fps.";
    return false;
  }
  if (!model_config->GetValue("query_tf_inverval_seconds",
                              &_query_tf_inverval_seconds)) {
    AERROR << "TLPreprocessorSubnode Failed to find Conf: "
           << "query_tf_inverval_seconds.";
    return false;
  }

  // init preprocessor
  if (!init_preprocessor()) {
    AERROR << "TLPreprocessorSubnode init failed.";
    return false;
  }

  // init hd_map
  if (!init_hdmap()) {
    AERROR << "TLPreprocessorSubnode Failed to init hdmap";
    return false;
  }
  // init projection
  if (!_projection.init()) {
    AERROR << "TLPreprocessorSubnode init projection failed.";
    return false;
  }

  using common::adapter::AdapterManager;
  CHECK(AdapterManager::GetImageLong())
  << "TLPreprocessorSubnode init failed.ImageLong is not initialized.";
  common::adapter::AdapterManager::AddImageLongCallback(&TLPreprocessorSubnode::sub_long_focus_camera,
                                                        this);
  CHECK(AdapterManager::GetImageShort())
  << "TLPreprocessorSubnode init failed.ImageShort is not initialized.";
  common::adapter::AdapterManager::AddImageShortCallback(&TLPreprocessorSubnode::sub_short_focus_camera,
                                                         this);
  return true;
}

bool TLPreprocessorSubnode::init_shared_data() {

  CHECK_NOTNULL(shared_data_manager_);

  const std::string preprocessing_data_name("TLPreprocessingData");
  _preprocessing_data = dynamic_cast<TLPreprocessingData *>(
      shared_data_manager_->GetSharedData(preprocessing_data_name));
  if (_preprocessing_data == nullptr) {
    AERROR << "TLPreprocessorSubnode failed to get shared data instance "
           << preprocessing_data_name;
    return false;
  }
  AINFO << "TLPreprocessorSubnode init shared data. name:" << _preprocessing_data->name();
  return true;
}

bool TLPreprocessorSubnode::init_preprocessor() {
  _preprocessor.reset(new TLPreprocessor);
  if (!_preprocessor) {
    AERROR << "TLPreprocessorSubnode new preprocessor failed";
    return false;
  }
  if (!_preprocessor->init()) {
    AERROR << "TLPreprocessorSubnode init preprocessor failed";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::init_hdmap() {
  _hd_map = HDMapInput::instance();
  if (_hd_map == nullptr) {
    AERROR << "TLPreprocessorSubnode get hdmap failed.";
    return false;
  }
  if (!_hd_map->Init()) {
    AERROR << "TLPreprocessorSubnode init hd-map failed.";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::add_data_and_publish_event(
    const std::shared_ptr<ImageLights> &data,
    const CameraId &camera_id,
    double timestamp) {
  // add data down-stream
  std::string device_id = kCameraIdToStr.at(camera_id);
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id, &key)) {
    AERROR << "TLPreprocessorSubnode gen share data key failed. ts:"
           << GLOG_TIMESTAMP(timestamp);
    return false;
  }

  if (!_preprocessing_data->Add(key, data)) {
    AERROR << "TLPreprocessorSubnode push data into shared_data failed.";
    data->image.reset();
    return false;
  }

  // pub events
  for (size_t i = 0; i < this->pub_meta_events_.size(); ++i) {
    const EventMeta &event_meta = this->pub_meta_events_[i];

    Event event;
    event.event_id = event_meta.event_id;
    event.reserve = device_id;
    event.timestamp = timestamp;
    this->event_manager_->Publish(event);
  }

  return true;
}

void TLPreprocessorSubnode::sub_long_focus_camera(const sensor_msgs::Image &msg) {
  std::shared_ptr<sensor_msgs::Image> img = common::adapter::AdapterManager::GetImageLong()->GetLatestObservedPtr();
  sub_camera_image(img, LONG_FOCUS);
}

void TLPreprocessorSubnode::sub_short_focus_camera(const sensor_msgs::Image &msg) {
  std::shared_ptr<sensor_msgs::Image> img = common::adapter::AdapterManager::GetImageShort()->GetLatestObservedPtr();
  sub_camera_image(img, SHORT_FOCUS);
}

void TLPreprocessorSubnode::sub_camera_image(
    const std::shared_ptr<sensor_msgs::Image> msg, CameraId camera_id) {
  const double sub_camera_image_start_ts = TimeUtil::GetCurrentTime();
  PERF_FUNCTION();
  std::shared_ptr<Image> image(new Image);
  cv::Mat cv_mat;
  double timestamp = 0.0;
  if (msg->header.stamp.toSec() < FLAGS_min_valid_ts_in_seconds ||
      msg->header.stamp.toSec() > FLAGS_max_valid_ts_in_seconds) {
    LOG(WARNING) << "TLPreprocessorSubnode rev bad image. "
                 << "ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec())
                 << ", which should be in [" << FLAGS_min_valid_ts_in_seconds << ","
                 << FLAGS_max_valid_ts_in_seconds << "]" << " camera_id:" << camera_id;
    return;
  }
  timestamp = msg->header.stamp.toSec();
  image->Init(timestamp, camera_id, msg);

  // update last timestamp when receiving a image
  _last_sub_camera_image_ts[camera_id] = timestamp;

  uint64_t timestamp_int64 = TimestampDouble2Int64(msg->header.stamp.toSec());
  timestamp_int64 += _s_camera_ts_last_3_digits[camera_id];

  AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << kCameraIdToStr.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec());

  bool camera_is_working = false;
  if (!_preprocessor->get_camera_is_working_flag(camera_id, &camera_is_working)) {
    AINFO << "get_camera_is_working_flag failed, ts: "
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
  }
  if (!camera_is_working) {
    // 检查相机投影配置，如果投影配置有误（如标定文件问题等），不标记当前相机为使用状态
    if (_projection.has_camera(camera_id) &&
        !_preprocessor->set_camera_is_working_flag(camera_id, true)) {
      AINFO << "set_camera_is_working_flag failed, ts: "
            << GLOG_TIMESTAMP(image->ts())
            << ", camera_id: " << kCameraIdToStr.at(camera_id);
    }
  }

  // 将原 sub_tf 的处理放到图像的 callback 里
  add_cached_camera_selection(timestamp);

  // 根据最大处理帧率和上一帧处理时间，来判断是否跳过当前帧
  _proc_interval_seconds = 1.0 / _max_process_image_fps;
  if (_last_proc_image_ts > 0.0 &&
      sub_camera_image_start_ts - _last_proc_image_ts < _proc_interval_seconds) {
    AINFO << "skip current image, img_ts: " << GLOG_TIMESTAMP(timestamp)
          << " , sub_camera_image_start_ts: " << GLOG_TIMESTAMP(sub_camera_image_start_ts)
          << " , _last_proc_image_ts: " << GLOG_TIMESTAMP(_last_proc_image_ts)
          << " , _proc_interval_seconds: " << GLOG_TIMESTAMP(_proc_interval_seconds);
    return;
  }

  // sync image and publish data
  const double before_sync_image_ts = TimeUtil::GetCurrentTime();
  std::shared_ptr<ImageLights> data(new ImageLights);
  bool should_pub = false;
  if (!_preprocessor->sync_image(image, image->ts(), camera_id, &data, &should_pub)) {
    AINFO << "sync image failed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  } else {
    AINFO << "sync image succeed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
  }
  const double sync_image_latency = TimeUtil::GetCurrentTime() - before_sync_image_ts;

  // CarOS Monitor 异常，图像时间与系统时间相差较大
  size_t max_cached_image_lights_array_size = 0;
  _preprocessor->get_max_cached_image_lights_array_size(&max_cached_image_lights_array_size);
  // tf 频率实际为 200Hz, 0.005 秒一帧，一共缓存了 max_cached_image_lights_array_size * 0.005 时间的 tf 信息
  double image_sys_ts_diff_threshold = max_cached_image_lights_array_size * 0.005;
  if (fabs(data->diff_image_sys_ts) > image_sys_ts_diff_threshold) {
    std::string debug_string = "";
    debug_string += ("diff_image_sys_ts:" + std::to_string(data->diff_image_sys_ts));
    debug_string += (",camera_id:" + kCameraIdToStr.at(camera_id));
    debug_string += (",camera_ts:" + std::to_string(timestamp));

    AWARN << "image_ts - system_ts(in seconds): "
          << std::to_string(data->diff_image_sys_ts)
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

  // verify lights projection
  // 根据图像时间戳再次查定位和灯，更新 data
  if (!verify_lights_projection(timestamp, camera_id, &data)) {
    AINFO << "TLPreprocessorSubnode verify_lights_projection on image failed, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << kCameraIdToStr.at(camera_id);
    return;
  }

  // 记录处理当前帧的时间
  _last_proc_image_ts = TimeUtil::GetCurrentTime();

  // convert rosmsg to cv::Mat
  const double before_rosmsg_to_cv_mat_ts = TimeUtil::GetCurrentTime();
  const double rosmsg_to_cv_mat_latency =
      TimeUtil::GetCurrentTime() - before_rosmsg_to_cv_mat_ts;

  data->preprocess_receive_timestamp = sub_camera_image_start_ts;
  data->preprocess_send_timestamp = TimeUtil::GetCurrentTime();
  if (add_data_and_publish_event(data, camera_id, image->ts())) {
    //_preprocessor->set_last_output_ts(image->ts());
    _preprocessor->set_last_pub_camera_id(camera_id);
    AINFO << "TLPreprocessorSubnode::sub_camera_image msg_time: "
          << GLOG_TIMESTAMP(image->ts())
          << " sync_image_latency: " << sync_image_latency * 1000 << " ms."
          << " rosmsg_to_cv_mat_latency: " << rosmsg_to_cv_mat_latency * 1000 << " ms."
          << " sub_camera_image_latency: " <<
          (TimeUtil::GetCurrentTime() -
              sub_camera_image_start_ts) * 1000 << " ms."
          << " camera_id: " << kCameraIdToStr.at(camera_id)
          << " number of lights: " << data->lights->size();
  }

}

bool TLPreprocessorSubnode::get_car_pose(const double ts, CarPose *pose) {
  Eigen::Matrix4d pose_matrix;
  // TODO:: query pose
  //if (!_velodyne2world_trans.query_pos(ts, &pose_matrix)) {
  //  AERROR << "TLPreprocessorSubnode failed to query pose ts:" << GLOG_TIMESTAMP(ts);
  //  return false;
  //}
  GetVelodyneTrans(ts, &pose_matrix);
  if (!pose->set_pose(pose_matrix)) {
    AERROR << "TLPreprocessorSubnode failed to init ts:" << GLOG_TIMESTAMP(ts)
           << " pose:" << pose_matrix;
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::verify_lights_projection(
    const double &ts,
    const CameraId &camera_id,
    std::shared_ptr<ImageLights> *image_lights) {
  // get car pose
  CarPose pose;
  if (!get_car_pose(ts, &pose)) {
    AERROR << "verify_lights_projection failed to get car pose, ts:"
           << GLOG_TIMESTAMP(ts);
    return false;
  }

  // get signals
  std::vector<apollo::hdmap::Signal> signals;
  double last_signals_ts = 0.0;
  double valid_hdmap_interval = 0.0;
  _preprocessor->get_last_signals_ts(&last_signals_ts);
  _preprocessor->get_valid_hdmap_interval(&valid_hdmap_interval);
  if (!_hd_map->GetSignals(pose.pose(), &signals)) {
    if (ts - last_signals_ts < valid_hdmap_interval) {
      _preprocessor->get_last_signals(&signals);
      AWARN << "verify_lights_projection failed to get signals info. Use last info\n"
            << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << pose;
    } else {
      AERROR << "verify_lights_projection failed to get signals info. "
             << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << pose;
      return false;
    }
  } else {
    _preprocessor->set_last_signals(signals);
    _preprocessor->set_last_signals_ts(ts);
  }

  bool projections_outside_all_images = false;
  CameraId selected_camera_id = CameraId::UNKNOWN;
  if (!_preprocessor->select_camera_by_lights_projection(
      ts, pose, signals, _projection, _s_image_borders, image_lights,
      &projections_outside_all_images, &selected_camera_id)) {
    AINFO << "_preprocessor->select_camera_by_lights_projection failed";
    return false;
  }

  if (camera_id != selected_camera_id) {
    AINFO << "verify_lights_projection selected_camera_id: "
          << kCameraIdToStr.at(selected_camera_id)
          << ", cached camera_id: " << kCameraIdToStr.at(camera_id)
          << ", image_ts: " << GLOG_TIMESTAMP(ts)
          << "; do not use this image.";
    return false;
  }

  return true;
}

void TLPreprocessorSubnode::add_cached_camera_selection(double ts) {
  const double current_ts = TimeUtil::GetCurrentTime();
  if (_last_query_tf_ts > 0.0 && current_ts - _last_query_tf_ts < _query_tf_inverval_seconds) {
    AINFO << "skip current tf msg, img_ts: " << GLOG_TIMESTAMP(ts)
          << " , _last_query_tf_ts: " << GLOG_TIMESTAMP(_last_query_tf_ts);
    return;
  }
  _last_query_tf_ts = current_ts;

  // get pose
  CarPose pose;
  if (!get_car_pose(ts, &pose)) {
    AERROR << "add_cached_camera_selection failed to get car pose, ts:"
           << GLOG_TIMESTAMP(ts);
    return;
  }
  auto pos_x = std::to_string(pose.pose()(0, 3));
  auto pos_y = std::to_string(pose.pose()(1, 3));
  AINFO << "add_cached_camera_selection get position (x, y): "
        << " (" << pos_x << ", " << pos_y << ").";

  // get signals
  std::vector<apollo::hdmap::Signal> signals;
  double last_signals_ts = 0.0;
  double valid_hdmap_interval = 0.0;
  _preprocessor->get_last_signals_ts(&last_signals_ts);
  _preprocessor->get_valid_hdmap_interval(&valid_hdmap_interval);
  if (!_hd_map->GetSignals(pose.pose(), &signals)) {
    if (ts - last_signals_ts < valid_hdmap_interval) {
      _preprocessor->get_last_signals(&signals);
      AWARN << "add_cached_camera_selection failed to get signals info. "
            << "Now use last info. ts:" << GLOG_TIMESTAMP(ts) << " pose:" << pose;
    } else {
      AERROR << "add_cached_camera_selection failed to get signals info. "
             << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << pose;
    }
  } else {
    _preprocessor->set_last_signals(signals);
    _preprocessor->set_last_signals_ts(ts);
  }

  bool projections_outside_all_images = false;
  if (!_preprocessor->add_cached_lights_projections(
      pose, signals, _projection, TLPreprocessorSubnode::_s_image_borders, ts,
      &projections_outside_all_images)) {
    AERROR << "add_cached_lights_projections failed, ts: " << GLOG_TIMESTAMP(ts);
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: " << GLOG_TIMESTAMP(ts);
  }
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
