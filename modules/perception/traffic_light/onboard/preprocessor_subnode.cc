// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/12 16:55:49
// @file: preprocessor_subnode.cpp
// @brief: preprocessor_subnode definition.
//
#include "module/perception/traffic_light/onboard/preprocessor_subnode.h"

#include <map>
#include <functional>
#include <string>
#include <vector>
#include <traffic_light/base/utils.h>
#include "ros/meta_stats.h"
//#include "roslibmetric/metric_handle.h"
#include "module/perception/traffic_light/onboard/rosmetrichandle_singleton.h"

#include "lib/base/macros.h"
#include "lib/base/file_util.h"
#include "lib/config_manager/config_manager.h"
#include "onboard/hdmap_input.h"
#include "onboard/event_manager.h"
#include "onboard/shared_data_manager.h"
#include "onboard/types.h"

DEFINE_double(min_valid_ts_in_seconds,
0.0,
"min valid timestamp, if ts < min_valid_ts_in_seconds image will be skipped.");
DEFINE_double(max_valid_ts_in_seconds,
FLT_MAX,
"max valid timestamp, if ts > max_valid_ts_in_seconds image will be skipped.");

using ::adu::perception::base::Singleton;

namespace adu {
namespace perception {
namespace traffic_light {

std::map<int, int> TLPreprocessorSubnode::_s_camera_ts_last_3_digits = {
    {static_cast<int>(CameraId::LONG_FOCUS), 222},
    {static_cast<int>(CameraId::SHORT_FOCUS), 111},
    {static_cast<int>(CameraId::NARROW_FOCUS), 444},
    {static_cast<int>(CameraId::WIDE_FOCUS), 333}
};

std::map<int, std::string> TLPreprocessorSubnode::_s_camera_names = {
    {static_cast<int>(CameraId::LONG_FOCUS), "long_focus_camera"},
    {static_cast<int>(CameraId::SHORT_FOCUS), "short_focus_camera"},
    {static_cast<int>(CameraId::NARROW_FOCUS), "12mm_focus_camera"},
    {static_cast<int>(CameraId::WIDE_FOCUS), "2mm_focus_camera"}
};

std::map<int, int> TLPreprocessorSubnode::_s_image_borders = {
    {static_cast<int>(CameraId::LONG_FOCUS), 100},
    {static_cast<int>(CameraId::SHORT_FOCUS), 100},
    {static_cast<int>(CameraId::NARROW_FOCUS), 100},
    {static_cast<int>(CameraId::WIDE_FOCUS), 100}
};
std::string TLPreprocessorSubnode::_s_debug_roi_relative_pos = "unknown";

TLPreprocessorSubnode::TLPreprocessorSubnode() {
}

bool TLPreprocessorSubnode::init_internal() {

  if (!init_shared_data()) {
    AERROR << "TLPreprocessorSubnode init failed. Shared Data init failed.";
    return false;
  }

  config_manager::ConfigManager *config_manager
      = base::Singleton<config_manager::ConfigManager>::get();
  std::string model_name("TLPreprocessorSubnode");
  const config_manager::ModelConfig *model_config(nullptr);
  if (!config_manager->get_model_config(model_name, &model_config)) {
    AERROR << "TLPreprocessorSubnode not found model: " << model_name;
    return false;
  }
  if (!model_config->get_value("max_process_image_fps",
                               &_max_process_image_fps)) {
    AERROR << "TLPreprocessorSubnode Failed to find Conf: "
           << "max_process_image_fps.";
    return false;
  }
  if (!model_config->get_value("sub_tf_inverval_seconds",
                               &_sub_tf_inverval_seconds)) {
    AERROR << "TLPreprocessorSubnode Failed to find Conf: "
           << "sub_tf_inverval_seconds.";
    return false;
  }

  // init preprocessor
  if (!init_preprocessor()) {
    AERROR << "TLPreprocessorSubnode init failed.";
    return false;
  }

  // parse reserve fileds
  std::map<std::string, std::string> reserve_field;
  if (!onboard::SubnodeHelper::parse_reserve_field(_reserve, &reserve_field)) {
    AERROR << "TLPreprocessorSubnode Failed to parse reserve filed."
           << " reserve:" << _reserve;
    return false;
  }

  // init TF module
  if (!init_transform_input(reserve_field)) {
    AERROR << "TLPreprocessorSubnode Failed to init transform input: " << _reserve;
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
           << " reserve:" << _reserve;
    return false;
  }
  // set short focus camera
  if (!init_subscriber(reserve_field, SHORT_FOCUS,
                       &TLPreprocessorSubnode::sub_short_focus_camera)) {
    AERROR << "TLPreprocessorSubnode init failed. init short focus camera."
           << " reserve:" << _reserve;
    return false;
  }

  // set 2mm focus camera
  if (!init_subscriber(reserve_field, WIDE_FOCUS,
                       &TLPreprocessorSubnode::sub_2mm_focus_camera)) {
    AERROR << "TLPreprocessorSubnode init failed. init 2mm focus camera."
           << " reserve:" << _reserve;
    return false;
  }

  // set 12mm focus camera
  if (!init_subscriber(reserve_field, NARROW_FOCUS,
                       &TLPreprocessorSubnode::sub_12mm_focus_camera)) {
    AERROR << "TLPreprocessorSubnode init failed. init 12mm focus camera."
           << " reserve:" << _reserve;
    return false;
  }

  // init tf subscriber
  if (!init_tf_subscriber(&TLPreprocessorSubnode::sub_tf)) {
    AERROR << "TLPreprocessorSubnode init failed. init tf."
           << " reserve:" << _reserve;
    return false;
  }

  // CarOS Monitor 异常
  // check if camera init. succeed
  const int num_camera_ids = static_cast<int>(CameraId::CAMERA_ID_COUNT) - 1;
  for (int i = 0; i < num_camera_ids; ++i) {
    CameraId cam_id = static_cast<CameraId>(i);
    if (!_projection.has_camera(cam_id)) {
      RosMetricHandleSingleton *ros_mh = Singleton<RosMetricHandleSingleton>::get();
      // std::string debug_string = "TLPreprocessorSubnode projection doesn't has camera: " +
      //         CAMERA_ID_TO_STR.at(cam_id) +
      //         ". Check if camera parameters files exist or not.";
      std::string debug_string = "";
      debug_string += ("camera_id:" + CAMERA_ID_TO_STR.at(cam_id));
      ros_mh->throw_exception(1, 101, debug_string);
      XLOG(WARN) << "TLPreprocessorSubnode projection doesn't has camera: "
                 << CAMERA_ID_TO_STR.at(cam_id)
                 << ". Check if camera parameters files exist or not."
                 << "debug_string: " << debug_string;

      // 长焦、短焦相机必须要有
      if (cam_id == LONG_FOCUS || cam_id == SHORT_FOCUS) {
        return false;
      }
    }
  }

  return true;
}

TLPreprocessorSubnode::~TLPreprocessorSubnode() {

}

bool TLPreprocessorSubnode::init_shared_data() {

  CHECK_NOTNULL(_shared_data_manager);

  const std::string preprocessing_data_name("TLPreprocessingData");
  _preprocessing_data = dynamic_cast<TLPreprocessingData *>(
      _shared_data_manager->get_shared_data(preprocessing_data_name));
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
  _hd_map = base::Singleton<onboard::HDMapInput>::get();
  if (_hd_map == nullptr) {
    AERROR << "TLPreprocessorSubnode get hdmap failed.";
    return false;
  }
  if (!_hd_map->init()) {
    AERROR << "TLPreprocessorSubnode init hd-map failed.";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::init_subscriber(
    const std::map<std::string, std::string> &fields,
    const CameraId &camera_id,
    void(TLPreprocessorSubnode::*callback)(const sensor_msgs::ImageConstPtr &)) {
  auto camera_name = _s_camera_names.at(camera_id);
  auto key = camera_name + "_source_type";
  auto citer = fields.find(key);
  if (citer == fields.end()) {
    AERROR << "Failed to find field:" << key << ", reserve: " << _reserve;
    return false;
  }
  onboard::IoStreamType source_type =
      static_cast<onboard::IoStreamType>(atoi((citer->second).c_str()));

  key = camera_name + "_source_name";
  citer = fields.find(key);
  if (citer == fields.end()) {
    AERROR << "Failed to find field:" << key << ", reserve: " << _reserve;
    return false;
  }
  const std::string &source_name = citer->second;

  std::string new_source_name = source_name;
  if (source_type == onboard::FILE_SOURCE) {
    new_source_name = base::FileUtil::get_absolute_path(_work_root_dir,
                                                        source_name);
  }

  // set source topic name
  _camera_topic_names[static_cast<int>(camera_id)] = new_source_name;


  // use to check whether camera is working
  _last_sub_camera_image_ts[static_cast<int>(camera_id)] = 0.0;

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
  std::string device_id = CAMERA_ID_TO_STR.at(camera_id);
  std::string key;
  if (!onboard::SubnodeHelper::produce_shared_data_key(timestamp, device_id, &key)) {
    AERROR << "TLPreprocessorSubnode gen share data key failed. ts:"
           << GLOG_TIMESTAMP(timestamp);
    return false;
  }

  if (!_preprocessing_data->add(key, data)) {
    AERROR << "TLPreprocessorSubnode push data into shared_data failed.";
    data->image.reset();
    return false;
  }

  // pub events
  for (size_t i = 0; i < this->_pub_meta_events.size(); ++i) {
    const onboard::EventMeta &event_meta = this->_pub_meta_events[i];

    onboard::Event event;
    event.event_id = event_meta.event_id;
    event.reserve = device_id;
    event.timestamp = timestamp;
    this->_event_manager->publish(event);
  }

  return true;
}

void TLPreprocessorSubnode::sub_long_focus_camera(const sensor_msgs::ImageConstPtr &msg) {
  sub_camera_image(msg, LONG_FOCUS);
}

void TLPreprocessorSubnode::sub_short_focus_camera(const sensor_msgs::ImageConstPtr &msg) {
  sub_camera_image(msg, SHORT_FOCUS);
}

void TLPreprocessorSubnode::sub_2mm_focus_camera(const sensor_msgs::ImageConstPtr &msg) {
  sub_camera_image(msg, WIDE_FOCUS);
}

void TLPreprocessorSubnode::sub_12mm_focus_camera(const sensor_msgs::ImageConstPtr &msg) {
  sub_camera_image(msg, NARROW_FOCUS);
}

void TLPreprocessorSubnode::sub_camera_image(
    const sensor_msgs::ImageConstPtr &msg, CameraId camera_id) {
  const double sub_camera_image_start_ts = base::TimeUtil::get_current_time();
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
  image->init(timestamp, camera_id, msg);

  // update last timestamp when receiving a image
  _last_sub_camera_image_ts[camera_id] = timestamp;

  uint64_t timestamp_int64 = ts_double_2_int64(msg->header.stamp.toSec());
  timestamp_int64 += _s_camera_ts_last_3_digits[camera_id];
  ros::MetaInfo info;
  info.camera_timestamp = timestamp_int64;
  info.lidar_timestamp = 0;
  ros::MetaStats::instance()->record_receive(info, _camera_topic_names[camera_id]);

  AINFO << "TLPreprocessorSubnode received a image msg"
        << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id)
        << ", ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec());

  bool camera_is_working = false;
  if (!_preprocessor->get_camera_is_working_flag(camera_id, &camera_is_working)) {
    AINFO << "get_camera_is_working_flag failed, ts: "
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
    return;
  }
  if (!camera_is_working) {
    // 检查相机投影配置，如果投影配置有误（如标定文件问题等），不标记当前相机为使用状态
    if (_projection.has_camera(camera_id) &&
        !_preprocessor->set_camera_is_working_flag(camera_id, true)) {
      AINFO << "set_camera_is_working_flag failed, ts: "
            << GLOG_TIMESTAMP(image->ts())
            << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
    }
  }

#if 1
  // 根据最大处理帧率和上一帧处理时间，来判断是否跳过当前帧
  _proc_interval_seconds = 1.0 / _max_process_image_fps;
  if (_last_proc_image_ts > 0.0 &&
      (timestamp > _last_proc_image_ts &&
          timestamp - _last_proc_image_ts < _proc_interval_seconds)) {
    AINFO << "skip current image, ts: " << GLOG_TIMESTAMP(timestamp)
          << " , _last_proc_image_ts: " << GLOG_TIMESTAMP(_last_proc_image_ts)
          << " , _proc_interval_seconds: " << GLOG_TIMESTAMP(_proc_interval_seconds);
    return;
  }
#endif

  // sync image and publish data
  const double before_sync_image_ts = base::TimeUtil::get_current_time();
  std::shared_ptr<ImageLights> data(new ImageLights);
  bool should_pub = false;
  if (!_preprocessor->sync_image(image, image->ts(), camera_id, &data, &should_pub)) {
    AINFO << "sync image failed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
  } else {
    AINFO << "sync image succeed ts: " << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
  }
  const double sync_image_latency = base::TimeUtil::get_current_time() - before_sync_image_ts;

  // CarOS Monitor 异常，图像时间与系统时间相差较大
  size_t max_cached_image_lights_array_size = 0;
  _preprocessor->get_max_cached_image_lights_array_size(&max_cached_image_lights_array_size);
  // tf 频率实际为 200Hz, 0.005 秒一帧，一共缓存了 max_cached_image_lights_array_size * 0.005 时间的 tf 信息
  double image_sys_ts_diff_threshold = max_cached_image_lights_array_size * 0.005;
  if (fabs(data->diff_image_sys_ts) > image_sys_ts_diff_threshold) {
    RosMetricHandleSingleton *ros_mh = Singleton<RosMetricHandleSingleton>::get();
    std::string debug_string = "";
    debug_string += ("diff_image_sys_ts:" + std::to_string(data->diff_image_sys_ts));
    debug_string += (",camera_id:" + CAMERA_ID_TO_STR.at(camera_id));
    debug_string += (",camera_ts:" + std::to_string(timestamp));
    ros_mh->throw_exception(1, 100, debug_string);

    XLOG(WARN) << "image_ts - system_ts(in seconds): "
               << std::to_string(data->diff_image_sys_ts)
               << ". Check if image timestamp drifts."
               << ", camera_id: " + CAMERA_ID_TO_STR.at(camera_id)
               << ", debug_string: " << debug_string;
  }

  if (!should_pub) {
    AINFO << "TLPreprocessorSubnode not publish image, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
    return;
  }

  // verify lights projection
  // 根据图像时间戳再次查定位和灯，更新 data
  if (!verify_lights_projection(timestamp, camera_id, &data)) {
    AINFO << "TLPreprocessorSubnode verify_lights_projection on image failed, ts:"
          << GLOG_TIMESTAMP(image->ts())
          << ", camera_id: " << CAMERA_ID_TO_STR.at(camera_id);
    return;
  }

  // 记录被处理图像的时间
  _last_proc_image_ts = timestamp;

  // convert rosmsg to cv::Mat
  const double before_rosmsg_to_cv_mat_ts = base::TimeUtil::get_current_time();
  //if (!rosmsg_to_cv_mat(msg, data->image->mutable_mat())) {
  //    AERROR << "TLPreprocessorSubnode rosmsg_to_cv_mat failed. "
  //                << "CameraId:" << camera_id << " ts:" << timestamp;
  //    return;
  //}
  const double rosmsg_to_cv_mat_latency =
      base::TimeUtil::get_current_time() - before_rosmsg_to_cv_mat_ts;

  data->preprocess_receive_timestamp = sub_camera_image_start_ts;
  data->preprocess_send_timestamp = base::TimeUtil::get_current_time();
  if (add_data_and_publish_event(data, camera_id, image->ts())) {
    //_preprocessor->set_last_output_ts(image->ts());
    _preprocessor->set_last_pub_camera_id(camera_id);
    AINFO << "TLPreprocessorSubnode::sub_camera_image msg_time: "
          << GLOG_TIMESTAMP(image->ts())
          << " sync_image_latency: " << sync_image_latency * 1000 << " ms."
          << " rosmsg_to_cv_mat_latency: " << rosmsg_to_cv_mat_latency * 1000 << " ms."
          << " sub_camera_image_latency: " <<
          (base::TimeUtil::get_current_time() -
              sub_camera_image_start_ts) * 1000 << " ms."
          << " camera_id: " << CAMERA_ID_TO_STR.at(camera_id)
          << " number of lights: " << data->lights->size();
  }

}

bool TLPreprocessorSubnode::rosmsg_to_image(const CameraId camera_id,
                                            const sensor_msgs::ImageConstPtr &msg,
                                            Image *image) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding.compare("rgb8") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding.compare("8UC3") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "8UC3");
    } else {
      AERROR << "TLPreprocessorSubnode get unknown image format. "
             << "format:" << msg->encoding;
      return false;
    }
  } catch (const cv_bridge::Exception &e) {
    AERROR << "TLPreprocessorSubnode trans msg to image failed." << e.what();
    return false;
  }

  if (msg->header.stamp.toSec() < FLAGS_min_valid_ts_in_seconds ||
      msg->header.stamp.toSec() > FLAGS_max_valid_ts_in_seconds) {
    LOG(WARNING) << "TLPreprocessorSubnode rev bad image. "
                 << "ts:" << GLOG_TIMESTAMP(msg->header.stamp.toSec())
                 << ", which should be in [" << FLAGS_min_valid_ts_in_seconds << ","
                 << FLAGS_max_valid_ts_in_seconds << "]" << " camera_id:" << camera_id;
    return false;
  }

  if (!image->init(msg->header.stamp.toSec(), camera_id, cv_ptr->image)) {
    AERROR << "TLPreprocessorSubnode load image failed.";
    return false;
  }
  return true;
}

bool TLPreprocessorSubnode::rosmsg_to_cv_mat(const sensor_msgs::ImageConstPtr &msg,
                                             cv::Mat *mat) {
  PERF_FUNCTION();
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding.compare("rgb8") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding.compare("8UC3") == 0) {
      cv_ptr = cv_bridge::toCvShare(msg, "8UC3");
    } else {
      AERROR << "TLPreprocessorSubnode get unknown image format. "
             << "format:" << msg->encoding;
      return false;
    }
  } catch (const cv_bridge::Exception &e) {
    AERROR << "TLPreprocessorSubnode trans msg to cv::Mat failed." << e.what();
    return false;
  }
  *mat = cv_ptr->image;

  return true;
}

bool TLPreprocessorSubnode::init_tf_subscriber(
    void (TLPreprocessorSubnode::*callback)(const tf2_msgs::TFMessageConstPtr &)) {
  // register subscriber
  if (!_tf_stream_input.register_subscriber(onboard::ROSMSG_SOURCE, "/tf", callback, this)) {
    AERROR << "TLPreprocessorSubnode Failed to register tf input stream.";

    return false;
  }

  AINFO << "TLPreprocessorSubnode Init tf subscriber successfully.";

  return true;
}

void TLPreprocessorSubnode::sub_tf(const tf2_msgs::TFMessageConstPtr &msg) {
  PERF_FUNCTION();
  double ts = 0.0;
  std::string child_frame_id = "";
  parse_tf_msg_timestamp(msg, &ts);
  parse_tf_msg_child_frame_id(msg, &child_frame_id);

  if (child_frame_id != "perception_localization_100hz") {
    AINFO << "sub_tf skipping msg, child_frame_id: " << child_frame_id
          << ", ts: " << GLOG_TIMESTAMP(ts);
    return;
  }

#if 0
  // check interval between 2 consecutive /tf msgs
  if (_last_sub_tf_ts > 0.0 && ts - _last_sub_tf_ts > 0.1) {
      XLOG(WARN) << "/tf msgs may be lost, current tf ts: " << GLOG_TIMESTAMP(ts)
              << ", _last_sub_tf_ts: " << GLOG_TIMESTAMP(_last_sub_tf_ts)
              << ", diff_tf_ts: " << GLOG_TIMESTAMP(ts - _last_sub_tf_ts);
  }
  _last_sub_tf_ts = ts;
#endif

#if 1
  {
    if (_last_sub_tf_ts > 0.0 && ts - _last_sub_tf_ts < _sub_tf_inverval_seconds) {
      // XLOG(WARN) << "/tf msgs may be lost, current tf ts: " << GLOG_TIMESTAMP(ts)
      //         << ", _last_sub_tf_ts: " << GLOG_TIMESTAMP(_last_sub_tf_ts)
      //         << ", diff_tf_ts: " << GLOG_TIMESTAMP(ts - _last_sub_tf_ts);
      AINFO << "skip current tf msg, ts: " << GLOG_TIMESTAMP(ts)
            << " , _last_sub_tf_ts: " << GLOG_TIMESTAMP(_last_sub_tf_ts);
      return;
    }
    _last_sub_tf_ts = ts;
  }
#endif

#if 0
  double last_no_signals_ts = 0.0;
  double no_signals_interval_seconds = 0.0;
  _preprocessor->get_last_no_signals_ts(&last_no_signals_ts);
  _preprocessor->get_no_signals_interval_seconds(&no_signals_interval_seconds);
  if (last_no_signals_ts > 0.0 && ts - last_no_signals_ts < no_signals_interval_seconds) {
      AINFO << "sub_tf skipping msg, ts: " << GLOG_TIMESTAMP(ts)
              << ", last_no_signals_ts: " << GLOG_TIMESTAMP(last_no_signals_ts)
              << ", (ts - last_no_signals_ts): " << GLOG_TIMESTAMP(ts - last_no_signals_ts);
      return;
  }
#endif

  // get pose
  CarPose pose;
  if (!get_car_pose(ts, &pose)) {
    AERROR << "sub_tf failed to get car pose, ts:" << GLOG_TIMESTAMP(ts);
    return;
  }
  auto pos_x = std::to_string(pose.get_pose()(0, 3));
  auto pos_y = std::to_string(pose.get_pose()(1, 3));
  AINFO << "sub_tf get position (x, y): " << " (" << pos_x << ", " << pos_y << ").";

  // get signals
  std::vector<adu::common::hdmap::Signal> signals;
  double last_signals_ts = 0.0;
  double valid_hdmap_interval = 0.0;
  _preprocessor->get_last_signals_ts(&last_signals_ts);
  _preprocessor->get_valid_hdmap_interval(&valid_hdmap_interval);
  if (!_hd_map->get_signals(pose.get_position(), &signals)) {
    if (ts - last_signals_ts < valid_hdmap_interval) {
      _preprocessor->get_last_signals(&signals);
      XLOG(WARN) << "sub_tf failed to get signals info. Now use last info\n"
                 << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << pose;
    } else {
      AERROR << "sub_tf failed to get signals info. "
             << "ts:" << GLOG_TIMESTAMP(ts) << " pose:" << pose;
    }
  } else {
    _preprocessor->set_last_signals(signals);
    _preprocessor->set_last_signals_ts(ts);
  }

  // 检查每个相机是否正常
  // 超过 1 秒没收到图像，认为相机挂了
  bool camera_ok = true;
  std::string no_image_camera_names = "";
  for (const auto &pr : _last_sub_camera_image_ts) {
    CameraId cam_id = static_cast<CameraId>(pr.first);
    double last_sub_camera_ts = pr.second;
    if (/*last_sub_camera_ts > 0.0 && */ts - last_sub_camera_ts > 1.0) {
      _preprocessor->set_camera_is_working_flag(cam_id, false);
      XLOG(WARN) << "camera is probably not working"
                 << ", current ts: " << GLOG_TIMESTAMP(ts)
                 << " , last_sub_camera_ts: " << GLOG_TIMESTAMP(last_sub_camera_ts)
                 << ", camera_id: " << CAMERA_ID_TO_STR.at(cam_id);
      camera_ok = false;
      no_image_camera_names += (" " + CAMERA_ID_TO_STR.at(cam_id));
    }
  }

  // CarOS Monitor 监控
  // 相机超过 1 秒未接收到图像
  if (!camera_ok) {
    RosMetricHandleSingleton *ros_mh = Singleton<RosMetricHandleSingleton>::get();
    std::string debug_string = "";
    debug_string += ("no_image_camera_names:" + no_image_camera_names);
    ros_mh->throw_exception(1, 103, debug_string);

    XLOG(WARN) << "[" + no_image_camera_names + " ] camera not receive images"
               << ", check whether camera is working."
               << ", debug_string: " << debug_string;
  }

  bool projections_outside_all_images = false;
  if (!_preprocessor->add_cached_lights_projections(
      pose, signals, _projection, TLPreprocessorSubnode::_s_image_borders, ts,
      &projections_outside_all_images)) {
    AERROR << "add_cached_lights_projections failed, ts: " << GLOG_TIMESTAMP(ts)
           << ", child_frame_id: " << child_frame_id;
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: " << GLOG_TIMESTAMP(ts)
          << ", child_frame_id: " << child_frame_id;
  }

  // CarOS Monitor 监控
  // 灯投影出所有图像
  if (projections_outside_all_images) {
    XLOG(WARN) << "TLPreprocessorSubnode lights projections outside all images, "
               << "ts: " << GLOG_TIMESTAMP(ts);

    RosMetricHandleSingleton *ros_mh = Singleton<RosMetricHandleSingleton>::get();
    std::string debug_string = "description:lights' projections are outside all images";
    ros_mh->throw_exception(1, 104, debug_string);
  }
}

bool TLPreprocessorSubnode::parse_tf_msg_timestamp(
    const tf2_msgs::TFMessageConstPtr &msg, double *timestamp) {
  std::stringstream ss;
  ss << *msg;
  std::string msg_str = ss.str();

  auto stamp_pos = msg_str.find("stamp");
  if (stamp_pos == std::string::npos) {
    return false;
  }
  auto colon_pos = msg_str.find(":", stamp_pos + 5);
  if (colon_pos == std::string::npos) {
    return false;
  }
  auto newline_pos = msg_str.find("\n", colon_pos);
  if (newline_pos == std::string::npos) {
    return false;
  }

  auto ts_str = msg_str.substr(colon_pos + 1, newline_pos - colon_pos - 1);
  *timestamp = std::stod(ts_str);

  return true;
}

bool TLPreprocessorSubnode::parse_tf_msg_child_frame_id(
    const tf2_msgs::TFMessageConstPtr &msg, std::string *child_frame_id) {
  std::stringstream ss;
  ss << *msg;
  std::string msg_str = ss.str();

  auto child_frame_id_pos = msg_str.find("child_frame_id");
  if (child_frame_id_pos == std::string::npos) {
    return false;
  }
  auto colon_pos = msg_str.find(":", child_frame_id_pos + 14);
  if (colon_pos == std::string::npos) {
    return false;
  }
  auto newline_pos = msg_str.find("\n", colon_pos);
  if (newline_pos == std::string::npos) {
    return false;
  }

  *child_frame_id = msg_str.substr(colon_pos + 2, newline_pos - colon_pos - 2);

  return true;
}

bool TLPreprocessorSubnode::get_car_pose(const double ts, CarPose *pose) {
  Eigen::Matrix4d pose_matrix;
  if (!_velodyne2world_trans.query_pos(ts, &pose_matrix)) {
    AERROR << "TLPreprocessorSubnode failed to query pose ts:" << GLOG_TIMESTAMP(ts);
    return false;
  }
  if (!pose->init(pose_matrix)) {
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
  std::vector<adu::common::hdmap::Signal> signals;
  double last_signals_ts = 0.0;
  double valid_hdmap_interval = 0.0;
  _preprocessor->get_last_signals_ts(&last_signals_ts);
  _preprocessor->get_valid_hdmap_interval(&valid_hdmap_interval);
  if (!_hd_map->get_signals(pose.get_position(), &signals)) {
    if (ts - last_signals_ts < valid_hdmap_interval) {
      _preprocessor->get_last_signals(&signals);
      XLOG(WARN) << "verify_lights_projection failed to get signals info. Use last info\n"
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
          << CAMERA_ID_TO_STR.at(selected_camera_id)
          << ", cached camera_id: " << CAMERA_ID_TO_STR.at(camera_id)
          << ", image_ts: " << GLOG_TIMESTAMP(ts)
          << "; do not use this image.";
    return false;
  }

  return true;
}

REGISTER_SUBNODE(TLPreprocessorSubnode);
} // namespace traffic_light
} // namespace perception
} // namespace adu
