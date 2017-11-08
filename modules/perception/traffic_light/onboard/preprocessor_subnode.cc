// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/12 16:55:49
// @file: preprocessor_subnode.cpp
// @brief: preprocessor_subnode definition.
//
#include "modules/perception/traffic_light/onboard/preprocessor_subnode.h"

#include <map>
#include <functional>
#include <string>
#include <vector>
#include <modules/perception/traffic_light/base/utils.h>
#include "ros/meta_stats.h"

#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/types.h"

DEFINE_double(min_valid_ts_in_seconds, 0.0,
"min valid timestamp, if ts < min_valid_ts_in_seconds image will be skipped.");
DEFINE_double(max_valid_ts_in_seconds, FLT_MAX,
              "max valid timestamp, if ts > max_valid_ts_in_seconds image will be skipped.");

namespace apollo {
namespace perception {
namespace traffic_light {

std::map<CameraId, int> TLPreprocessorSubnode::_s_camera_ts_last_3_digits = {
    {CameraId::LONG_FOCUS, 222},
    {CameraId::SHORT_FOCUS, 111},
    {CameraId::NARROW_FOCUS, 444},
    {CameraId::WIDE_FOCUS, 333}
};

std::map<int, std::string> TLPreprocessorSubnode::_s_camera_names = {
    {CameraId::LONG_FOCUS, "long_focus_camera"},
    {CameraId::SHORT_FOCUS, "short_focus_camera"},
    {CameraId::NARROW_FOCUS, "12mm_focus_camera"},
    {CameraId::WIDE_FOCUS, "2mm_focus_camera"}
};

std::map<CameraId, int> TLPreprocessorSubnode::_s_image_borders = {
    {CameraId::LONG_FOCUS, 100},
    {CameraId::SHORT_FOCUS, 100},
    {CameraId::NARROW_FOCUS, 100},
    {CameraId::WIDE_FOCUS, 100}
};
std::string TLPreprocessorSubnode::_s_debug_roi_relative_pos = "unknown";

TLPreprocessorSubnode::TLPreprocessorSubnode() {
}

bool TLPreprocessorSubnode::InitInternal() {

  if (!init_shared_data()) {
    AERROR << "TLPreprocessorSubnode init failed. Shared Data init failed.";
    return false;
  }

  ConfigManager *config_manager
      = ConfigManager::instance();
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

  // parse reserve fileds
  std::map<std::string, std::string> reserve_field;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field)) {
    AERROR << "TLPreprocessorSubnode Failed to parse reserve filed."
           << " reserve:" << reserve_;
    return false;
  }

  // init TF module
  if (!init_transform_input(reserve_field)) {
    AERROR << "TLPreprocessorSubnode Failed to init transform input: " << reserve_;
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

  // set long focus camera
  if (!init_subscriber(reserve_field, LONG_FOCUS,
                       &TLPreprocessorSubnode::sub_long_focus_camera)) {
    AERROR << "TLPreprocessorSubnode init failed. init long focus camera."
           << " reserve:" << reserve_;
    return false;
  }
  // set short focus camera
  if (!init_subscriber(reserve_field, SHORT_FOCUS,
                       &TLPreprocessorSubnode::sub_short_focus_camera)) {
    AERROR << "TLPreprocessorSubnode init failed. init short focus camera."
           << " reserve:" << reserve_;
    return false;
  }

  return true;
}

TLPreprocessorSubnode::~TLPreprocessorSubnode() {

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

bool TLPreprocessorSubnode::init_transform_input(
    const std::map<std::string, std::string> &fields) {
  // init velodyne to world tf
  const auto citer = fields.find("velodyne2world_trans");
  if (citer == fields.end()) {
    AERROR << "Failed to find velodyne2world_trans conf.";
    return false;
  }
  AINFO << "[velodyne2world] :" << citer->second;
  const std::string &velodyne2world = citer->second;
  if (!_velodyne2world_trans.init(velodyne2world)) {
    AERROR << "TLPreprocessorSubnode failed to init velodyne2world_trans";
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

bool TLPreprocessorSubnode::init_subscriber(
    const std::map<std::string, std::string> &fields,
    const CameraId &camera_id,
    void(TLPreprocessorSubnode::*callback)(const sensor_msgs::ImageConstPtr &)) {
  auto camera_name = _s_camera_names[camera_id];
  auto key = camera_name + "_source_type";
  auto citer = fields.find(key);
  if (citer == fields.end()) {
    AERROR << "Failed to find field:" << key << ", reserve: " << reserve_;
    return false;
  }
  IoStreamType source_type =
      static_cast<IoStreamType>(atoi((citer->second).c_str()));

  key = camera_name + "_source_name";
  citer = fields.find(key);
  if (citer == fields.end()) {
    AERROR << "Failed to find field:" << key << ", reserve: " << reserve_;
    return false;
  }
  const std::string &source_name = citer->second;

  std::string new_source_name = source_name;

  // set source topic name
  _camera_topic_names[camera_id] = new_source_name;


  // use to check whether camera is working
  _last_sub_camera_image_ts[camera_id] = 0.0;

  // register subscriber
  if (!_stream_input.register_subscriber(source_type, new_source_name, callback, this)) {
    AERROR << "Failed to register input stream. [type: " << source_type
           << "] [name: " << new_source_name << "].";
    return false;
  }

  AINFO << "Init subscriber successfully, source_type: " << source_type
        << ", source_name: " << new_source_name;
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
    this->event_manager_->publish(event);
  }

  return true;
}

void TLPreprocessorSubnode::sub_long_focus_camera(const sensor_msgs::ImageConstPtr &msg) {
  sub_camera_image(msg, LONG_FOCUS);
}

void TLPreprocessorSubnode::sub_short_focus_camera(const sensor_msgs::ImageConstPtr &msg) {
  sub_camera_image(msg, SHORT_FOCUS);
}

void TLPreprocessorSubnode::sub_camera_image(
    const sensor_msgs::ImageConstPtr &msg, CameraId camera_id) {
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
  ros::MetaInfo info;
  info.camera_timestamp = timestamp_int64;
  info.lidar_timestamp = 0;
  ros::MetaStats::instance()->record_receive(info, _camera_topic_names[camera_id]);

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
  //if (!rosmsg_to_cv_mat(msg, data->image->mutable_mat())) {
  //    AERROR << "TLPreprocessorSubnode rosmsg_to_cv_mat failed. "
  //                << "CameraId:" << camera_id << " ts:" << timestamp;
  //    return;
  //}
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
  if (!_velodyne2world_trans.query_pos(ts, &pose_matrix)) {
    AERROR << "TLPreprocessorSubnode failed to query pose ts:" << GLOG_TIMESTAMP(ts);
    return false;
  }
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
  if (!_hd_map->get_signals(pose.pose(), &signals)) {
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
  if (!_hd_map->get_signals(pose.pose(), &signals)) {
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

REGISTER_SUBNODE(TLPreprocessorSubnode);
} // namespace traffic_light
} // namespace perception
} // namespace adu
