/******************************************************************************
* Copyright 2018 The Apollo Authors. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the License);
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an AS IS BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*****************************************************************************/
#include "modules/perception/onboard/component/trafficlights_perception_component.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>

#include "cybertron/common/log.h"
#include "cybertron/time/time.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time_util.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"
#include "modules/perception/lib/singleton/singleton.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/lib/utils/time_util.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace perception {
namespace onboard {
typedef apollo::perception::TrafficLightDetection::CameraID TLCamID;
using apollo::perception::common::SensorManager;

static int GetGpuId(
    const apollo::perception::camera::CameraPerceptionInitOptions& options) {
  apollo::perception::camera::app::TrafficLightParam trafficlight_param;
  std::string work_root = "";
  apollo::perception::camera::GetCybertronWorkRoot(&work_root);
  std::string config_file =
    lib::FileUtil::GetAbsolutePath(options.root_dir,
                                   options.conf_file);
  config_file = lib::FileUtil::GetAbsolutePath(work_root, config_file);
  if (!lib::ParseProtobufFromFile<camera::app::TrafficLightParam>(
        config_file,
        &trafficlight_param)) {
    AERROR << "Read config failed: " << config_file;
    return -1;
  }
  if (trafficlight_param.detector_param_size() == 0) {
    AERROR << "get gpu id failed. detector_param_size() == 0";
    return -1;
  }
  if (!trafficlight_param.has_gpu_id()) {
    AINFO << "gpu id not found.";
    return -1;
  }
  return trafficlight_param.gpu_id();
}

bool TrafficLightsPerceptionComponent::Init() {
  writer_ = node_->CreateWriter<apollo::perception::TrafficLightDetection>(
      "/perception/traffic_light");

  if (InitConfig() != cybertron::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitConfig failed.";
    return cybertron::FAIL;
  }

  if (InitAlgorithmPlugin() != cybertron::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitAlgorithmPlugin failed.";
    return cybertron::FAIL;
  }

  if (InitCameraListeners() != cybertron::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitCameraListeners failed.";
    return cybertron::FAIL;
  }

  if (InitCameraFrame() != cybertron::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitCameraFrame failed.";
    return cybertron::FAIL;
  }

  AINFO << "TrafficLight Preproc Init Success";
  return cybertron::SUCC;
}

int TrafficLightsPerceptionComponent::InitConfig() {
  apollo::perception::onboard::TrafficLight traffic_light_param;

  const std::string proto_path =
      "../production/conf/perception/camera_onboard/"
      "trafficlights_perception_component.config";

  if (!GetProtoConfig(&traffic_light_param)) {
    AINFO << "load trafficlights perception component proto param failed, "
        "file dir";
    return false;
  }

  tf2_frame_id_ = traffic_light_param.tl_tf2_frame_id();
  tf2_child_frame_id_ = traffic_light_param.tl_tf2_child_frame_id();
  tf2_timeout_second_ = traffic_light_param.tf2_timeout_second();
  AINFO << "tl_tf2_frame_id: " << tf2_frame_id_
           << " tl_tf2_child_frame_id: " << tf2_child_frame_id_
           << " tf2_buff_size: " << tf2_timeout_second_;

  std::string camera_names_str = "";
  camera_names_str = traffic_light_param.camera_names();
  boost::algorithm::split(camera_names_, camera_names_str,
      boost::algorithm::is_any_of(","));

  std::string camera_channel_names_str = "";
  camera_channel_names_str = traffic_light_param.camera_channel_names();
  boost::algorithm::split(input_camera_channel_names_,
      camera_channel_names_str,
      boost::algorithm::is_any_of(","));

  query_tf_interval_seconds_ = traffic_light_param.query_tf_interval_seconds();
  valid_hdmap_interval_ = traffic_light_param.valid_hdmap_interval();
  image_timestamp_offset_ = traffic_light_param.tl_image_timestamp_offset();
  AINFO << " _image_timestamp_offset: " << image_timestamp_offset_;

  max_process_image_fps_ = traffic_light_param.max_process_image_fps();
  proc_interval_seconds_ = 1.0 / max_process_image_fps_;
  AINFO << "_proc_interval_seconds: " << proc_interval_seconds_;

  image_sys_ts_diff_threshold_ =
    traffic_light_param.image_sys_ts_diff_threshold();
  preprocessor_init_options_.sync_interval_seconds =
    traffic_light_param.sync_interval_seconds();
  camera_perception_init_options_.root_dir =
    traffic_light_param.camera_traffic_light_perception_conf_dir();
  camera_perception_init_options_.conf_file =
    traffic_light_param.camera_traffic_light_perception_conf_file();
  default_image_border_size_ =
    traffic_light_param.default_image_border_size();

  simulation_channel_name_ = traffic_light_param.simulation_channel_name();
  traffic_light_output_channel_name_ =
    traffic_light_param.traffic_light_output_channel_name();

  return cybertron::SUCC;
}

int TrafficLightsPerceptionComponent::InitAlgorithmPlugin() {
  // init preprocessor
  preprocessor_.reset(new camera::TLPreprocessor);
  if (!preprocessor_) {
    AERROR << "TrafficLightsPerceptionComponent new preprocessor failed";
    return cybertron::FAIL;
  }

  preprocessor_init_options_.camera_names = camera_names_;
  if (!preprocessor_->Init(preprocessor_init_options_)) {
    AERROR << "TrafficLightsPerceptionComponent init preprocessor failed";
    return cybertron::FAIL;
  }
  const auto camera_names_by_descending_focal_len =
      preprocessor_->GetCameraNamesByDescendingFocalLen();
  if (camera_names_by_descending_focal_len.empty()) {
    AERROR << "empty camera_names in preprocessor";
    return cybertron::FAIL;
  }
  if (camera_names_.size() != input_camera_channel_names_.size() ||
      camera_names_.size() == 0) {
    AERROR << "invalid camera_names config";
    return cybertron::FAIL;
  }
  SensorManager* sensor_manager = lib::Singleton<SensorManager>::get_instance();
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    if (!sensor_manager->IsSensorExist(camera_names_[i])) {
      AERROR << ("sensor_name: " + camera_names_[i] + " not exists.");
      return cybertron::FAIL;
    }

    // init transform wrappers
    std::string tf2_camera_child_frame_id =
        sensor_manager->GetFrameId(camera_names_[i]);
    std::shared_ptr<TransformWrapper> trans_wrapper(new TransformWrapper);
    trans_wrapper->Init(tf2_camera_child_frame_id);
    camera2world_trans_wrapper_map_[camera_names_[i]] = trans_wrapper;

    if (camera_names_[i] == camera_names_by_descending_focal_len.back()) {
      image_border_sizes_[camera_names_[i]] = 0;
    } else {
      image_border_sizes_[camera_names_[i]] = default_image_border_size_;
    }
  }

  // init hdmaps
  hd_map_ = lib::Singleton<map::HDMapInput>::get_instance();
  if (hd_map_ == nullptr) {
    AERROR << "PreprocessComponent get hdmap failed.";
    return cybertron::FAIL;
  }

  if (!hd_map_->Init()) {
    AERROR << "PreprocessComponent init hd-map failed.";
    return cybertron::FAIL;
  }

  camera_perception_init_options_.use_cybertron_work_root = true;
  traffic_light_pipeline_.reset(new camera::TrafficLightCameraPerception);
  if (!traffic_light_pipeline_->Init(camera_perception_init_options_)) {
    AERROR << "camera_obstacle_pipeline_->Init() failed";
    return cybertron::FAIL;
  }

  return cybertron::SUCC;
}

int TrafficLightsPerceptionComponent::InitCameraListeners() {
  // init camera listeners
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    const auto &camera_name = camera_names_[i];
    const auto &camera_channel_name = input_camera_channel_names_[i];
    const std::string camera_listener_name = "tl_" + camera_name + "_listener";

    typedef const std::shared_ptr<
        apollo::drivers::Image> ImageMsgType;
    std::function<void(const ImageMsgType&)> sub_camera_callback =
        std::bind(&TrafficLightsPerceptionComponent::OnReceiveImage,
                  this, std::placeholders::_1, camera_name);
    auto sub_camera_reader = node_->CreateReader(
        camera_channel_name, sub_camera_callback);
    last_sub_camera_image_ts_[camera_name] = 0.0;
  }

  return cybertron::SUCC;
}

int TrafficLightsPerceptionComponent::InitCameraFrame() {
  data_provider_init_options_.image_height = image_height_;
  data_provider_init_options_.image_width = image_width_;
  int gpu_id = GetGpuId(camera_perception_init_options_);
  if (gpu_id == -1) {
    return cybertron::FAIL;
  }
  data_provider_init_options_.device_id = gpu_id;
  AINFO << "trafficlights data_provider_init_options_.device_id: "
      << data_provider_init_options_.device_id;
  data_provider_init_options_.do_undistortion = enable_undistortion_;

  // init data_providers for each camrea
  for (const auto &camera_name : camera_names_) {
    data_provider_init_options_.sensor_name = camera_name;
    std::shared_ptr<camera::DataProvider> data_provider(
        new camera::DataProvider);
    if (!data_provider->Init(data_provider_init_options_)) {
      AERROR << "trafficlights init data_provider failed. "
          << " camera_name: " << camera_name;
      return cybertron::FAIL;
    }
    data_providers_map_[camera_name] = data_provider;
  }

  return cybertron::SUCC;
}

void TrafficLightsPerceptionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> msg,
    const std::string& camera_name) {
  std::lock_guard<std::mutex> lck(mutex_);
  double receive_img_timestamp = lib::TimeUtil::GetCurrentTime();
  double image_msg_ts = msg->measurement_time();
  image_msg_ts += image_timestamp_offset_;
  last_sub_camera_image_ts_[camera_name] = image_msg_ts;

  {
    const double cur_time = lib::TimeUtil::GetCurrentTime();
    const double start_latency =
        (cur_time - msg->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:TrafficLights:Start:msg_time["
      << GLOG_TIMESTAMP(msg->measurement_time()) << "]:cur_time["
      << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
      << "]";
  }

  const std::string perf_indicator = "trafficlights";
  PERCEPTION_PERF_BLOCK_START();
  if (!CheckCameraImageStatus(image_msg_ts,
                              check_image_status_interval_thresh_,
                              camera_name)) {
    AERROR << "CheckCameraImageStatus failed";
    return;
  }
  const auto check_camera_status_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "CheckCameraImageStatus");

  camera::TLPreprocessorOption preprocess_option;
  preprocess_option.image_borders_size = &image_border_sizes_;

  // query pose and signals, add cached camera selection by lights' projections
  if (!UpdateCameraSelection(image_msg_ts, preprocess_option, &frame_)) {
    AWARN << "add_cached_camera_selection failed, ts: "
             << std::to_string(image_msg_ts);
  }
  const auto update_camera_selection_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "UpdateCameraSelection");

  // skipping frame according to last proc image timestamp
  if (last_proc_image_ts_ > 0.0 &&
      receive_img_timestamp - last_proc_image_ts_ < proc_interval_seconds_) {
    AINFO << "skip current image, img_ts: " << std::to_string(image_msg_ts)
             << " , receive_img_timestamp: "
             << std::to_string(receive_img_timestamp)
             << " ,_last_proc_image_ts: " << std::to_string(last_proc_image_ts_)
             << " , _proc_interval_seconds: "
             << std::to_string(proc_interval_seconds_);
    SendSimulationMsg();
    return;
  }
  // sync image with cached projections
  bool sync_image_ok = preprocessor_->SyncInformation(image_msg_ts,
      camera_name);
  const auto sync_information_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "SyncInformation");

  if (!sync_image_ok) {
    AINFO << "PreprocessComponent not publish image, ts:"
             << std::to_string(image_msg_ts)
             << ", camera_name: " << camera_name;
    SendSimulationMsg();
    return;
  }

  // Fill camera frame
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  frame_.data_provider = data_providers_map_.at(camera_name).get();
  frame_.data_provider->FillImageData(image_height_,
                                     image_width_,
                                     reinterpret_cast<const uint8_t*>
                                     (msg->data().data()),
                                     msg->encoding());
  frame_.timestamp = image_msg_ts;
  const auto fill_image_data_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "FillImageData");

  // caros monitor -- image system time diff
  const auto &diff_image_sys_ts = image_msg_ts - receive_img_timestamp;
  if (fabs(diff_image_sys_ts) > image_sys_ts_diff_threshold_) {
    std::string metric_name = "perception traffic_light exception";
    std::string debug_string = "";
    debug_string += ("diff_image_sys_ts:" + std::to_string(diff_image_sys_ts));
    debug_string += (",camera_id:" + camera_name);
    debug_string += (",camera_ts:" + std::to_string(image_msg_ts));
    AWARN << "image_ts - system_ts(in seconds): "
             << std::to_string(diff_image_sys_ts)
             << ". Check if image timestamp drifts."
             << ", camera_id: " + camera_name
             << ", debug_string: " << debug_string;
  }

  if (!VerifyLightsProjection(image_msg_ts, preprocess_option, camera_name,
                              &frame_)) {
    AINFO << "VerifyLightsProjection on image failed, ts: "
             << std::to_string(image_msg_ts)
             << ", camera_name: " << camera_name
             << " last_query_tf_ts_: "
             << std::to_string(last_query_tf_ts_)
             << " need update_camera_selection immediately,"
             << " reset last_query_tf_ts_ to -1";
    last_query_tf_ts_ = -1.0;
  }
  const auto verify_lights_projection_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "VerifyLightsProjection");
  last_proc_image_ts_ = lib::TimeUtil::GetCurrentTime();

  AINFO << "start proc.";
  traffic_light_pipeline_->Perception(camera_perception_options_, &frame_);

  const auto traffic_lights_perception_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "TrafficLightsPerception");
  for (auto light : frame_.traffic_lights) {
    AINFO << "after tl pipeline " << light->id
             << " color " << static_cast<int>(light->status.color);
  }

  std::shared_ptr<apollo::perception::TrafficLightDetection> out_msg =
      std::make_shared<apollo::perception::TrafficLightDetection>();
  if (!TransformOutputMessage(&frame_, camera_name, &out_msg)) {
    AERROR << "transform_output_message failed, msg_time: "
              << GLOG_TIMESTAMP(msg->measurement_time());
    return;
  }

  // send msg
  writer_->Write(out_msg);

  SendSimulationMsg();

  const auto send_message_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
          "SendMessage");

  const auto total_time =
      static_cast<int64_t>((lib::TimeUtil::GetCurrentTime() -
          receive_img_timestamp) * 1e3);
  AINFO << "TrafficLightsPerception perf_info."
      << " number_of_lights: " << frame_.traffic_lights.size()
      << " check_camera_status_time: " << check_camera_status_time << " ms."
      << " update_camera_selection_time: "
      << update_camera_selection_time << " ms."
      << " sync_information_time: " << sync_information_time << " ms."
      << " fill_image_data_time: " << fill_image_data_time << " ms."
      << " verify_lights_projection_time: "
      << verify_lights_projection_time << " ms."
      << " traffic_lights_perception_time: "
      << traffic_lights_perception_time << " ms."
      << " send_message_time: " << send_message_time << " ms."
      << " total: " << total_time << " ms.";
  AINFO << out_msg->DebugString();
  {
    const double end_timestamp = lib::TimeUtil::GetCurrentTime();
    const double end_latency =
        (end_timestamp - msg->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:TrafficLights:End:msg_time["
      << GLOG_TIMESTAMP(msg->measurement_time()) << "]:cur_time["
      << GLOG_TIMESTAMP(end_timestamp) << "]:cur_latency[" << end_latency
      << "]";
  }
}

void TrafficLightsPerceptionComponent::GenerateTrafficLights(
    const std::vector<apollo::hdmap::Signal>& signals,
    std::vector<base::TrafficLightPtr> *traffic_lights) {
  traffic_lights->clear();
  for (auto signal : signals) {
    base::TrafficLightPtr light;
    light.reset(new base::TrafficLight);
    light->id = signal.id().id();
    for (int i = 0; i < signal.boundary().point_size(); ++i) {
      base::PointXYZID point;
      point.x = signal.boundary().point(i).x();
      point.y = signal.boundary().point(i).y();
      point.z = signal.boundary().point(i).z();
      light->region.points.push_back(point);
    }

    int cur_semantic = 0;
    // for (int i = 0; i < signal.control_direction_size(); i++) {
    //   cur_semantic |= 1 << signal.control_direction(i);
    // }
    light->semantic = cur_semantic;
    traffic_lights->push_back(light);
    stoplines_ = signal.stop_line();
  }
}

bool TrafficLightsPerceptionComponent::QueryPoseAndSignals(
    const double ts,
    camera::CarPose* pose,
    std::vector<apollo::hdmap::Signal>* signals) {
  PERCEPTION_PERF_FUNCTION();
  // get pose
  if (!GetCarPose(ts, pose)) {
    AINFO << "query_pose_and_signals failed to get car pose, ts:"
             << std::to_string(ts);
    return false;
  }
  auto pos_x = std::to_string(pose->getCarPose()(0, 3));
  auto pos_y = std::to_string(pose->getCarPose()(1, 3));
  AINFO << "query_pose_and_signals get position (x, y): "
           << " (" << pos_x << ", " << pos_y << ").";

  if (!hd_map_) {
    AERROR << "hd_map_ not init.";
    return false;
  }
  // get signals
  Eigen::Vector3d car_position = pose->getCarPosition();
  if (!hd_map_->GetSignals(car_position, forward_distance_to_query_signals,
                           signals)) {
    if (ts - last_signals_ts_ < valid_hdmap_interval_) {
      *signals = last_signals_;
      AWARN << "query_pose_and_signals failed to get signals info. "
               << "Now use last info. ts:" << std::to_string(ts) << " pose:"
               << *pose << " signals.size(): " << signals->size();
    } else {
      AERROR << "query_pose_and_signals failed to get signals info. "
                << "ts:" << std::to_string(ts) << " pose:" << *pose;
    }
  } else {
    AINFO << "query_pose_and_signals succeeded, signals.size(): "
             << signals->size();
    // here need mutex lock_guard, added at the beginning of OnReceiveImage()
    last_signals_ts_ = ts;
    last_signals_ = *signals;
  }
  return true;
}

bool TrafficLightsPerceptionComponent::VerifyLightsProjection(
    const double& ts, const camera::TLPreprocessorOption& option,
    const std::string& camera_name, camera::CameraFrame* frame) {
  PERCEPTION_PERF_FUNCTION();
  camera::CarPose pose;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(ts, &pose, &signals)) {
    AERROR << "query_pose_and_signals failed, ts: "
              << std::to_string(ts);
    // (*image_lights)->debug_info.is_pose_valid = false;
    return false;
  }

  GenerateTrafficLights(signals, &frame->traffic_lights);

  if (!preprocessor_->UpdateLightsProjection(pose, option, camera_name,
                                             &frame->traffic_lights)) {
    AWARN << "verify_lights_projection failed to update_lights_projection, "
             << " ts: " << std::to_string(ts);
    return false;
  }

  AINFO << "VerifyLightsProjection success " << frame->traffic_lights.size();

  return true;
}

bool TrafficLightsPerceptionComponent::UpdateCameraSelection(double timestamp,
      const camera::TLPreprocessorOption& option, camera::CameraFrame* frame) {
  PERCEPTION_PERF_FUNCTION();
  const double current_ts = lib::TimeUtil::GetCurrentTime();
  if (last_query_tf_ts_ > 0.0 &&
      current_ts - last_query_tf_ts_ < query_tf_interval_seconds_) {
    AINFO << "skip current tf msg, img_ts: " << std::to_string(timestamp)
             << " , last_query_tf_ts_: " << std::to_string(last_query_tf_ts_);
    return true;
  }
  AINFO << "start select camera";

  camera::CarPose pose;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(timestamp, &pose, &signals)) {
    AINFO << "query_pose_and_signals failed, ts: "
             << std::to_string(timestamp);
    return false;
  }
  last_query_tf_ts_ = current_ts;

  GenerateTrafficLights(signals, &frame->traffic_lights);
  AINFO << "hd map signals " << frame->traffic_lights.size();

  if (!preprocessor_->UpdateCameraSelection(
      pose, option, &frame->traffic_lights)) {
    AERROR << "add_cached_lights_projections failed, ts: "
              << std::to_string(timestamp);
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: "
             << std::to_string(timestamp);
  }

  for (auto &light : frame->traffic_lights) {
    AINFO << "x " << light->region.projection_roi.x
             << " y " << light->region.projection_roi.y
             << " w " << light->region.projection_roi.width
             << " h " << light->region.projection_roi.height;
  }
  return true;
}

bool TrafficLightsPerceptionComponent::CheckCameraImageStatus(double timestamp,
    double interval, const std::string& camera_name) {
  PERCEPTION_PERF_FUNCTION();
  bool camera_ok = true;
  std::string no_image_camera_names = "";
  for (const auto &pr : last_sub_camera_image_ts_) {
    const auto cam_name = pr.first;
    double last_sub_camera_ts = pr.second;
    // should be 0.0, change to 1 in case of float precision
    if (last_sub_camera_ts < 1.0 ||
        timestamp - last_sub_camera_ts > interval) {
      preprocessor_->SetCameraWorkingFlag(cam_name, false);
      AWARN << "camera is probably not working"
               << " , current ts: " << timestamp
               << " , last_sub_camera_ts: "
               << last_sub_camera_ts
               << " , camera_name: " << cam_name;
      camera_ok = false;
      AINFO << "camera status:" << camera_ok;
      no_image_camera_names += (" " + cam_name);
    }
  }

  bool is_camera_working = false;
  if (!preprocessor_->GetCameraWorkingFlag(camera_name, &is_camera_working)) {
    AERROR << "get_camera_is_working_flag ts: "
              << std::to_string(timestamp)
              << " camera_name: " << camera_name;
    return false;
  }
  if (!is_camera_working) {
    if (!preprocessor_->SetCameraWorkingFlag(camera_name, true)) {
      AERROR << "set_camera_is_working_flag ts: "
                << std::to_string(timestamp)
                << " camera_name: " << camera_name;
      return false;
    }
  }
  return true;
}

bool TrafficLightsPerceptionComponent::GetCarPose(
    const double timestamp, camera::CarPose *pose) {
  PERCEPTION_PERF_FUNCTION();
  Eigen::Matrix4d pose_matrix;
  // get pose car(gps) to world
  if (!GetPoseFromTF(timestamp, tf2_frame_id_, tf2_child_frame_id_,
                     &pose_matrix)) {
    AERROR << "get pose from tf failed, child_frame_id: "
              << tf2_child_frame_id_;
    return false;
  }
  if (!pose->Init(timestamp, pose_matrix)) {
    AERROR << "PreprocessComponent::get_car_pose failed, ts:"
              << std::to_string(timestamp)
              << " pose:" << pose_matrix;
    return false;
  }

  int state = 0;
  int ret = cybertron::FAIL;
  Eigen::Affine3d affine3d_trans;
  for (const auto &camera_name : camera_names_) {
    const auto trans_wrapper = camera2world_trans_wrapper_map_[camera_name];
    ret = trans_wrapper->GetSensor2worldTrans(timestamp, &affine3d_trans);
    pose_matrix = affine3d_trans.matrix();
    if (ret != cybertron::SUCC) {
      pose->ClearCameraPose(camera_name);
      AERROR << "get pose from tf failed, camera_name: "
                << camera_name;
    } else {
      pose->SetCameraPose(camera_name, pose_matrix);
      state += 1;
    }
  }
  return state > 0;
}

bool TrafficLightsPerceptionComponent::GetPoseFromTF(
    const double timestamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    Eigen::Matrix4d* pose_matrix) {
  PERCEPTION_PERF_FUNCTION();
  apollo::cybertron::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id,
                                query_time, tf2_timeout_second_, &err_string)) {
    AERROR << "Can not find transform. " << std::to_string(timestamp)
              << " frame_id: " << frame_id
              << " child_frame_id: " << child_frame_id
              << " Error info: " << err_string;
    return false;
  }
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform = tf2_buffer_->lookupTransform(
        frame_id, child_frame_id, query_time);
    Eigen::Translation3d translation(
        stamped_transform.transform().translation().x(),
        stamped_transform.transform().translation().y(),
        stamped_transform.transform().translation().z());
    Eigen::Quaterniond rotation(
        stamped_transform.transform().rotation().qw(),
        stamped_transform.transform().rotation().qx(),
        stamped_transform.transform().rotation().qy(),
        stamped_transform.transform().rotation().qz());
    *pose_matrix = (translation * rotation).matrix();
    ADEBUG << "get pose: " << *pose_matrix;
  }
  catch (tf2::TransformException &ex) {
    AERROR << ex.what();
    return false;
  }
  return true;
}

bool TrafficLightsPerceptionComponent::TransformOutputMessage(
    camera::CameraFrame* frame,
    const std::string& camera_name,
    std::shared_ptr<TrafficLightDetection>* out_msg) {
  PERCEPTION_PERF_FUNCTION();
  const std::map<std::string, TLCamID> CAMERA_ID_TO_TLCAMERA_ID = {
      {"onsemi_traffic", TrafficLightDetection::CAMERA_FRONT_LONG},
      {"onsemi_narrow", TrafficLightDetection::CAMERA_FRONT_NARROW},
      {"onsemi_obstacle", TrafficLightDetection::CAMERA_FRONT_SHORT},
      {"onsemi_wide", TrafficLightDetection::CAMERA_FRONT_WIDE}
  };

  const auto &lights = frame->traffic_lights;
  auto *header = (*out_msg)->mutable_header();
  double publish_time = apollo::cybertron::Time::Now().ToSecond();
  header->set_timestamp_sec(publish_time);  // message publishing time
  AINFO << "set header time sec:" << std::to_string(frame->timestamp);

  // sec -> nano-sec
  uint64_t ts_int64 = static_cast<uint64_t>(frame->timestamp * 1e9);
  header->set_camera_timestamp(ts_int64);

  if (CAMERA_ID_TO_TLCAMERA_ID.find(camera_name) ==
      CAMERA_ID_TO_TLCAMERA_ID.end()) {
    AERROR << "unknown camera_name: " << camera_name;
    return false;
  }
  (*out_msg)->set_camera_id(CAMERA_ID_TO_TLCAMERA_ID.at(camera_name));

  // add traffic light result
  for (size_t i = 0; i < lights.size(); i++) {
    apollo::perception::TrafficLight *light_result =
        (*out_msg)->add_traffic_light();
    light_result->set_id(lights.at(i)->id);
    light_result->set_confidence(lights.at(i)->status.confidence);
    light_result->set_color(static_cast<apollo::perception::TrafficLight_Color>
                            (lights.at(i)->status.color));
    light_result->set_blink(lights.at(i)->status.blink);
  }

  // set contain_lights
  (*out_msg)->set_contain_lights(lights.size() > 0);

  // add traffic light debug info
  if (!TransformDebugMessage(frame, out_msg)) {
    AERROR << "ProcComponent::Proc failed to transform debug msg.";
    return false;
  }

  return true;
}

void TrafficLightsPerceptionComponent::TransRect2Box(
    const base::RectI &rect,
    apollo::perception::TrafficLightBox* box) {
  box->set_x(rect.x);
  box->set_y(rect.y);
  box->set_width(rect.width);
  box->set_height(rect.height);
}

double TrafficLightsPerceptionComponent::stopline_distance(
    const Eigen::Matrix4d& cam_pose) {
  if (stoplines_.size() == 0) {
    AWARN << "compute car to stopline's distance failed(no stopline). "
             << "cam_pose:" << cam_pose;
    return -1;
  }
  const apollo::hdmap::Curve& stopline = stoplines_.Get(0);
  if (stopline.segment_size() == 0) {
    AWARN << "compute car to stopline's distance"
             << " failed(stopline has no segment line). "
             << "cam_pose:" << cam_pose
             << " stopline:" << stopline.ShortDebugString();
    return -1;
  }
  if (!stopline.segment(0).has_line_segment()) {
    AWARN << "compute car to stopline's distance "
             << "failed(stopline has no segment). "
             << "cam_pose:" << cam_pose
             << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  if (stopline.segment(0).line_segment().point_size() == 0) {
    AWARN << "compute car to stopline's distance "
             << "failed(stopline has no point). "
             << "cam_pose:" << cam_pose
             << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  Eigen::Vector3d stopline_pt(stopline.segment(0).line_segment().point(0).x(),
                              stopline.segment(0).line_segment().point(0).y(),
                              stopline.segment(0).line_segment().point(0).z());
  Eigen::Vector3d stopline_pt_cam = (cam_pose.inverse() * Eigen::Vector4d(
      stopline_pt(0), stopline_pt(1), stopline_pt(2), 1.0)).head(3);

  return stopline_pt_cam(2);
}

bool TrafficLightsPerceptionComponent::TransformDebugMessage(
  const camera::CameraFrame* frame,
  std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg) {
  PERCEPTION_PERF_FUNCTION();
  const auto &lights = frame->traffic_lights;
  // add traffic light debug info
  apollo::perception::TrafficLightDebug *light_debug =
      (*out_msg)->mutable_traffic_light_debug();

  // signal number
  light_debug->set_signal_num(lights.size());

  // Crop ROI
  if (lights.size() > 0 && lights.at(0)->region.debug_roi.size() > 0) {
    auto& crop_roi = lights.at(0)->region.debug_roi[0];
    auto tl_cropbox = light_debug->mutable_cropbox();
    TransRect2Box(crop_roi, tl_cropbox);
  }

  // Rectified ROI
  for (size_t i = 0; i < lights.size(); ++i) {
    auto& rectified_roi = lights.at(i)->region.detection_roi;
    auto tl_rectified_box = light_debug->add_box();
    TransRect2Box(rectified_roi, tl_rectified_box);
    tl_rectified_box->set_color(
        static_cast<TrafficLight_Color>(lights.at(i)->status.color));
    tl_rectified_box->set_selected(true);
  }

  // Projection ROI
  for (size_t i = 0; i < lights.size(); ++i) {
    auto& projection_roi = lights.at(i)->region.projection_roi;
    auto tl_projection_box = light_debug->add_box();
    TransRect2Box(projection_roi, tl_projection_box);
  }

  // debug ROI (candidate detection boxes)
  if (lights.size() > 0 && lights.at(0)->region.debug_roi.size() > 0) {
    for (size_t i = 1; i < lights.at(0)->region.debug_roi.size(); ++i) {
      auto& debug_roi = lights.at(0)->region.debug_roi[i];
      auto tl_debug_box = light_debug->add_box();
      TransRect2Box(debug_roi, tl_debug_box);
    }
  }

  // Crop ROI
  for (size_t i = 0; i < lights.size(); ++i) {
    auto& crop_roi = lights.at(i)->region.debug_roi[0];
    auto tl_crop_box = light_debug->add_crop_roi();
    TransRect2Box(crop_roi, tl_crop_box);
  }

  // Detection ROI
  for (size_t i = 0; i < lights.size(); ++i) {
    auto& rectified_roi = lights.at(i)->region.detection_roi;
    auto tl_rectified_box = light_debug->add_rectified_roi();
    TransRect2Box(rectified_roi, tl_rectified_box);
    tl_rectified_box->set_color(
        static_cast<TrafficLight_Color>(lights.at(i)->status.color));
    tl_rectified_box->set_selected(true);
  }

  // Projection ROI
  for (size_t i = 0; i < lights.size(); ++i) {
    auto& projection_roi = lights.at(i)->region.projection_roi;
    auto tl_projection_box = light_debug->add_projected_roi();
    TransRect2Box(projection_roi, tl_projection_box);
  }

  // debug ROI (candidate detection boxes)
  if (lights.size() > 0 && lights.at(0)->region.debug_roi.size() > 0) {
    for (size_t i = 1; i < lights.at(0)->region.debug_roi.size(); ++i) {
      auto& debug_roi = lights.at(0)->region.debug_roi[i];
      auto tl_debug_box = light_debug->add_debug_roi();
      TransRect2Box(debug_roi, tl_debug_box);
    }
  }
  if (lights.size() > 0) {
    camera::CarPose pose;
    if (GetCarPose(frame->timestamp, &pose)) {
      Eigen::Matrix4d cam_pose;
      pose.GetCameraPose("onsemi_traffic", &cam_pose);
      double distance = stopline_distance(cam_pose);
      light_debug->set_distance_to_stop_line(distance);
    } else {
      AERROR << "error occured in calc distance to stop line";
    }
  }

  return true;
}

void TrafficLightsPerceptionComponent::SendSimulationMsg() {
  auto out_msg = std::make_shared<TrafficLightDetection>();
  writer_->Write(out_msg);
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
