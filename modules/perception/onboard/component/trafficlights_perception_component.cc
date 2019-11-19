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

#include <boost/algorithm/string.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <limits>
#include <map>
#include <utility>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/time/time_util.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/lib/utils/time_util.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace perception {
namespace onboard {
using TLCamID = apollo::perception::TrafficLightDetection::CameraID;
using apollo::cyber::common::GetAbsolutePath;
using apollo::perception::common::SensorManager;

class TLInfo {
 public:
  cv::Scalar tl_color_;
  std::string tl_string_;
  std::string tl_string_ex_;
};

std::map<base::TLColor, TLInfo> s_tl_infos = {
    {base::TLColor::TL_UNKNOWN_COLOR,
     {cv::Scalar(255, 255, 255), "UNKNOWN", "UNKNOWN traffic light"}},
    {base::TLColor::TL_RED,
     {cv::Scalar(0, 0, 255), "RED", "RED traffic light"}},
    {base::TLColor::TL_GREEN,
     {cv::Scalar(0, 255, 0), "GREEN", "GREEN traffic light"}},
    {base::TLColor::TL_YELLOW,
     {cv::Scalar(0, 255, 255), "YELLOW", "YELLOW traffic light"}}};

static int GetGpuId(
    const apollo::perception::camera::CameraPerceptionInitOptions& options) {
  apollo::perception::camera::app::TrafficLightParam trafficlight_param;
  std::string work_root = apollo::perception::camera::GetCyberWorkRoot();
  std::string config_file =
      GetAbsolutePath(options.root_dir, options.conf_file);
  config_file = GetAbsolutePath(work_root, config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &trafficlight_param)) {
    AERROR << "Read config failed: " << config_file;
    return -1;
  }
  if (trafficlight_param.detector_param().empty()) {
    AERROR << "get gpu id failed. detector_param().empty()";
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
      "/apollo/perception/traffic_light");

  if (InitConfig() != cyber::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitConfig failed.";
    return false;
  }

  if (InitAlgorithmPlugin() != cyber::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitAlgorithmPlugin failed.";
    return false;
  }

  if (InitCameraFrame() != cyber::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitCameraFrame failed.";
    return false;
  }

  if (InitCameraListeners() != cyber::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitCameraListeners failed.";
    return false;
  }

  if (InitV2XListener() != cyber::SUCC) {
    AERROR << "TrafficLightsPerceptionComponent InitV2XListener failed.";
    return false;
  }

  if (FLAGS_start_visualizer) {
    AINFO << "TrafficLight Visualizer is ON";
  } else {
    AINFO << "TrafficLight Visualizer is OFF";
  }

  AINFO << "TrafficLight Preproc Init Success";
  return true;
}

int TrafficLightsPerceptionComponent::InitConfig() {
  apollo::perception::onboard::TrafficLight traffic_light_param;
  if (!GetProtoConfig(&traffic_light_param)) {
    AINFO << "load trafficlights perception component proto param failed";
    return cyber::FAIL;
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
  boost::algorithm::split(input_camera_channel_names_, camera_channel_names_str,
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
      static_cast<float>(traffic_light_param.sync_interval_seconds());
  camera_perception_init_options_.root_dir =
      traffic_light_param.camera_traffic_light_perception_conf_dir();
  camera_perception_init_options_.conf_file =
      traffic_light_param.camera_traffic_light_perception_conf_file();
  default_image_border_size_ = traffic_light_param.default_image_border_size();

  simulation_channel_name_ = traffic_light_param.simulation_channel_name();
  traffic_light_output_channel_name_ =
      traffic_light_param.traffic_light_output_channel_name();

  // v2x params
  v2x_trafficlights_input_channel_name_ =
      traffic_light_param.v2x_trafficlights_input_channel_name();
  v2x_sync_interval_seconds_ = traffic_light_param.v2x_sync_interval_seconds();
  max_v2x_msg_buff_size_ = traffic_light_param.max_v2x_msg_buff_size();
  v2x_msg_buffer_.set_capacity(max_v2x_msg_buff_size_);
  return cyber::SUCC;
}

int TrafficLightsPerceptionComponent::InitAlgorithmPlugin() {
  // init preprocessor
  preprocessor_.reset(new camera::TLPreprocessor);
  if (!preprocessor_) {
    AERROR << "TrafficLightsPerceptionComponent new preprocessor failed";
    return cyber::FAIL;
  }

  preprocessor_init_options_.camera_names = camera_names_;
  if (!preprocessor_->Init(preprocessor_init_options_)) {
    AERROR << "TrafficLightsPerceptionComponent init preprocessor failed";
    return cyber::FAIL;
  }
  const auto camera_names_by_descending_focal_len =
      preprocessor_->GetCameraNamesByDescendingFocalLen();
  if (camera_names_by_descending_focal_len.empty()) {
    AERROR << "empty camera_names in preprocessor";
    return cyber::FAIL;
  }
  if (camera_names_.size() != input_camera_channel_names_.size() ||
      camera_names_.empty()) {
    AERROR << "invalid camera_names config";
    return cyber::FAIL;
  }
  SensorManager* sensor_manager = SensorManager::Instance();
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    if (!sensor_manager->IsSensorExist(camera_names_[i])) {
      AERROR << ("sensor_name: " + camera_names_[i] + " not exists.");
      return cyber::FAIL;
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
  hd_map_ = map::HDMapInput::Instance();
  if (hd_map_ == nullptr) {
    AERROR << "PreprocessComponent get hdmap failed.";
    return cyber::FAIL;
  }

  if (!hd_map_->Init()) {
    AERROR << "PreprocessComponent init hd-map failed.";
    return cyber::FAIL;
  }

  camera_perception_init_options_.use_cyber_work_root = true;
  traffic_light_pipeline_.reset(new camera::TrafficLightCameraPerception);
  if (!traffic_light_pipeline_->Init(camera_perception_init_options_)) {
    AERROR << "camera_obstacle_pipeline_->Init() failed";
    return cyber::FAIL;
  }

  return cyber::SUCC;
}

int TrafficLightsPerceptionComponent::InitCameraListeners() {
  // init camera listeners
  for (size_t i = 0; i < camera_names_.size(); ++i) {
    const auto& camera_name = camera_names_[i];
    const auto& camera_channel_name = input_camera_channel_names_[i];
    const std::string camera_listener_name = "tl_" + camera_name + "_listener";

    typedef const std::shared_ptr<apollo::drivers::Image> ImageMsgType;
    std::function<void(const ImageMsgType&)> sub_camera_callback =
        std::bind(&TrafficLightsPerceptionComponent::OnReceiveImage, this,
                  std::placeholders::_1, camera_name);
    auto sub_camera_reader =
        node_->CreateReader(camera_channel_name, sub_camera_callback);
    last_sub_camera_image_ts_[camera_name] = 0.0;
  }

  return cyber::SUCC;
}

int TrafficLightsPerceptionComponent::InitV2XListener() {
  typedef const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData>
      V2XTrafficLightsMsgType;
  std::function<void(const V2XTrafficLightsMsgType&)> sub_v2x_tl_callback =
      std::bind(&TrafficLightsPerceptionComponent::OnReceiveV2XMsg, this,
                std::placeholders::_1);
  auto sub_v2x_reader = node_->CreateReader(
      v2x_trafficlights_input_channel_name_, sub_v2x_tl_callback);
  return cyber::SUCC;
}

int TrafficLightsPerceptionComponent::InitCameraFrame() {
  data_provider_init_options_.image_height = image_height_;
  data_provider_init_options_.image_width = image_width_;
  int gpu_id = GetGpuId(camera_perception_init_options_);
  if (gpu_id == -1) {
    return cyber::FAIL;
  }
  data_provider_init_options_.device_id = gpu_id;
  AINFO << "trafficlights data_provider_init_options_.device_id: "
        << data_provider_init_options_.device_id;
  data_provider_init_options_.do_undistortion = enable_undistortion_;

  // init data_providers for each camrea
  for (const auto& camera_name : camera_names_) {
    data_provider_init_options_.sensor_name = camera_name;
    std::shared_ptr<camera::DataProvider> data_provider(
        new camera::DataProvider);
    if (!data_provider->Init(data_provider_init_options_)) {
      AERROR << "trafficlights init data_provider failed. "
             << " camera_name: " << camera_name;
      return cyber::FAIL;
    }
    data_providers_map_[camera_name] = data_provider;
  }

  return cyber::SUCC;
}

void TrafficLightsPerceptionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> msg,
    const std::string& camera_name) {
  std::lock_guard<std::mutex> lck(mutex_);
  double receive_img_timestamp = apollo::common::time::Clock::NowInSeconds();
  double image_msg_ts = msg->measurement_time();
  image_msg_ts += image_timestamp_offset_;
  last_sub_camera_image_ts_[camera_name] = image_msg_ts;

  {
    const double cur_time = apollo::common::time::Clock::NowInSeconds();
    const double start_latency = (cur_time - msg->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:TrafficLights:Start:msg_time["
          << GLOG_TIMESTAMP(msg->measurement_time()) << "]:cur_time["
          << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
          << "]";
  }

  const std::string perf_indicator = "trafficlights";
  PERCEPTION_PERF_BLOCK_START();
  if (!CheckCameraImageStatus(image_msg_ts, check_image_status_interval_thresh_,
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
    AWARN << "add_cached_camera_selection failed, ts: " << image_msg_ts;
  }
  const auto update_camera_selection_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
                                               "UpdateCameraSelection");

  // skipping frame according to last proc image timestamp
  if (last_proc_image_ts_ > 0.0 &&
      receive_img_timestamp - last_proc_image_ts_ < proc_interval_seconds_) {
    AINFO << "skip current image, img_ts: " << image_msg_ts
          << " , receive_img_timestamp: " << receive_img_timestamp
          << " ,_last_proc_image_ts: " << last_proc_image_ts_
          << " , _proc_interval_seconds: " << proc_interval_seconds_;
    //    SendSimulationMsg();
    return;
  }
  // sync image with cached projections
  bool sync_image_ok =
      preprocessor_->SyncInformation(image_msg_ts, camera_name);
  const auto sync_information_time = PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      perf_indicator, "SyncInformation");

  if (!sync_image_ok) {
    AINFO << "PreprocessComponent not publish image, ts:" << image_msg_ts
          << ", camera_name: " << camera_name;
    //    SendSimulationMsg();
    return;
  }

  // Fill camera frame
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  frame_.data_provider = data_providers_map_.at(camera_name).get();
  frame_.data_provider->FillImageData(
      image_height_, image_width_,
      reinterpret_cast<const uint8_t*>(msg->data().data()), msg->encoding());
  frame_.timestamp = image_msg_ts;
  const auto fill_image_data_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator, "FillImageData");

  // caros monitor -- image system time diff
  const auto& diff_image_sys_ts = image_msg_ts - receive_img_timestamp;
  if (fabs(diff_image_sys_ts) > image_sys_ts_diff_threshold_) {
    const std::string metric_name = "perception traffic_light exception";
    const std::string debug_string =
        absl::StrCat("diff_image_sys_ts:", diff_image_sys_ts,
                     ",camera_id:", camera_name, ",camera_ts:", image_msg_ts);
    AWARN << "image_ts - system_ts(in seconds): " << diff_image_sys_ts
          << ". Check if image timestamp drifts."
          << ", camera_id: " + camera_name
          << ", debug_string: " << debug_string;
  }

  if (!VerifyLightsProjection(image_msg_ts, preprocess_option, camera_name,
                              &frame_)) {
    AINFO << "VerifyLightsProjection on image failed, ts: " << image_msg_ts
          << ", camera_name: " << camera_name
          << " last_query_tf_ts_: " << last_query_tf_ts_
          << " need update_camera_selection immediately,"
          << " reset last_query_tf_ts_ to -1";
    last_query_tf_ts_ = -1.0;
  }
  const auto verify_lights_projection_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
                                               "VerifyLightsProjection");
  last_proc_image_ts_ = apollo::common::time::Clock::NowInSeconds();

  AINFO << "start proc.";
  traffic_light_pipeline_->Perception(camera_perception_options_, &frame_);

  const auto traffic_lights_perception_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator,
                                               "TrafficLightsPerception");
  for (auto light : frame_.traffic_lights) {
    AINFO << "after tl pipeline " << light->id << " color "
          << static_cast<int>(light->status.color);
  }

  SyncV2XTrafficLights(&frame_);

  std::shared_ptr<apollo::perception::TrafficLightDetection> out_msg =
      std::make_shared<apollo::perception::TrafficLightDetection>();
  if (!TransformOutputMessage(&frame_, camera_name, &out_msg)) {
    AERROR << "transform_output_message failed, msg_time: "
           << GLOG_TIMESTAMP(msg->measurement_time());
    return;
  }

  // send msg
  writer_->Write(out_msg);

  //  SendSimulationMsg();

  const auto send_message_time =
      PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(perf_indicator, "SendMessage");

  const auto total_time = static_cast<int64_t>(
      (apollo::common::time::Clock::NowInSeconds() - receive_img_timestamp) *
      1e3);
  AINFO << "TrafficLightsPerception perf_info."
        << " number_of_lights: " << frame_.traffic_lights.size()
        << " check_camera_status_time: " << check_camera_status_time << " ms."
        << " update_camera_selection_time: " << update_camera_selection_time
        << " ms."
        << " sync_information_time: " << sync_information_time << " ms."
        << " fill_image_data_time: " << fill_image_data_time << " ms."
        << " verify_lights_projection_time: " << verify_lights_projection_time
        << " ms."
        << " traffic_lights_perception_time: " << traffic_lights_perception_time
        << " ms."
        << " send_message_time: " << send_message_time << " ms."
        << " total: " << total_time << " ms.";
  AINFO << out_msg->DebugString();
  {
    const double end_timestamp = apollo::common::time::Clock::NowInSeconds();
    const double end_latency = (end_timestamp - msg->measurement_time()) * 1e3;
    AINFO << "FRAME_STATISTICS:TrafficLights:End:msg_time["
          << GLOG_TIMESTAMP(msg->measurement_time()) << "]:cur_time["
          << GLOG_TIMESTAMP(end_timestamp) << "]:cur_latency[" << end_latency
          << "]";
  }
}

void TrafficLightsPerceptionComponent::OnReceiveV2XMsg(
    const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData> v2x_msg) {
  std::lock_guard<std::mutex> lck(mutex_);
  v2x_msg_buffer_.push_back(*v2x_msg);
}

void TrafficLightsPerceptionComponent::GenerateTrafficLights(
    const std::vector<apollo::hdmap::Signal>& signals,
    std::vector<base::TrafficLightPtr>* traffic_lights) {
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
    const double ts, camera::CarPose* pose,
    std::vector<apollo::hdmap::Signal>* signals) {
  PERCEPTION_PERF_FUNCTION();
  // get pose
  if (!GetCarPose(ts, pose)) {
    AINFO << "query_pose_and_signals failed to get car pose, ts:" << ts;
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
            << "Now use last info. ts:" << ts << " pose:" << *pose
            << " signals.size(): " << signals->size();
    } else {
      AERROR << "query_pose_and_signals failed to get signals info. "
             << "ts:" << ts << " pose:" << *pose;
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
    AERROR << "query_pose_and_signals failed, ts: " << ts;
    // (*image_lights)->debug_info.is_pose_valid = false;
    return false;
  }

  GenerateTrafficLights(signals, &frame->traffic_lights);

  if (!preprocessor_->UpdateLightsProjection(pose, option, camera_name,
                                             &frame->traffic_lights)) {
    AWARN << "verify_lights_projection failed to update_lights_projection, "
          << " ts: " << ts;
    return false;
  }

  AINFO << "VerifyLightsProjection success " << frame->traffic_lights.size();

  return true;
}

bool TrafficLightsPerceptionComponent::UpdateCameraSelection(
    double timestamp, const camera::TLPreprocessorOption& option,
    camera::CameraFrame* frame) {
  PERCEPTION_PERF_FUNCTION();
  const double current_ts = apollo::common::time::Clock::NowInSeconds();
  if (last_query_tf_ts_ > 0.0 &&
      current_ts - last_query_tf_ts_ < query_tf_interval_seconds_) {
    AINFO << "skip current tf msg, img_ts: " << timestamp
          << " , last_query_tf_ts_: " << last_query_tf_ts_;
    return true;
  }
  AINFO << "start select camera";

  camera::CarPose pose;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(timestamp, &pose, &signals)) {
    AINFO << "query_pose_and_signals failed, ts: " << timestamp;
    return false;
  }
  last_query_tf_ts_ = current_ts;

  GenerateTrafficLights(signals, &frame->traffic_lights);
  AINFO << "hd map signals " << frame->traffic_lights.size();

  if (!preprocessor_->UpdateCameraSelection(pose, option,
                                            &frame->traffic_lights)) {
    AERROR << "add_cached_lights_projections failed, ts: " << timestamp;
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: " << timestamp;
  }

  for (auto& light : frame->traffic_lights) {
    AINFO << "x " << light->region.projection_roi.x << " y "
          << light->region.projection_roi.y << " w "
          << light->region.projection_roi.width << " h "
          << light->region.projection_roi.height;
  }
  return true;
}

bool TrafficLightsPerceptionComponent::CheckCameraImageStatus(
    double timestamp, double interval, const std::string& camera_name) {
  PERCEPTION_PERF_FUNCTION();
  bool camera_ok = true;
  std::string no_image_camera_names = "";
  for (const auto& pr : last_sub_camera_image_ts_) {
    const auto cam_name = pr.first;
    double last_sub_camera_ts = pr.second;
    // should be 0.0, change to 1 in case of float precision
    if (last_sub_camera_ts < 1.0 || timestamp - last_sub_camera_ts > interval) {
      preprocessor_->SetCameraWorkingFlag(cam_name, false);
      AWARN << "camera is probably not working"
            << " , current ts: " << timestamp
            << " , last_sub_camera_ts: " << last_sub_camera_ts
            << " , camera_name: " << cam_name;
      camera_ok = false;
      AINFO << "camera status:" << camera_ok;
      no_image_camera_names += (" " + cam_name);
    }
  }

  bool is_camera_working = false;
  if (!preprocessor_->GetCameraWorkingFlag(camera_name, &is_camera_working)) {
    AERROR << "get_camera_is_working_flag ts: " << timestamp
           << " camera_name: " << camera_name;
    return false;
  }
  if (!is_camera_working) {
    if (!preprocessor_->SetCameraWorkingFlag(camera_name, true)) {
      AERROR << "set_camera_is_working_flag ts: " << timestamp
             << " camera_name: " << camera_name;
      return false;
    }
  }
  return true;
}

bool TrafficLightsPerceptionComponent::GetCarPose(const double timestamp,
                                                  camera::CarPose* pose) {
  PERCEPTION_PERF_FUNCTION();
  Eigen::Matrix4d pose_matrix;
  // get pose car(gps) to world
  if (!GetPoseFromTF(timestamp, tf2_frame_id_, tf2_child_frame_id_,
                     &pose_matrix)) {
    AERROR << "get pose from tf failed, child_frame_id: "
           << tf2_child_frame_id_;
    return false;
  }
  pose->timestamp_ = timestamp;
  pose->pose_ = pose_matrix;

  int state = 0;
  bool ret = true;
  Eigen::Affine3d affine3d_trans;
  for (const auto& camera_name : camera_names_) {
    const auto trans_wrapper = camera2world_trans_wrapper_map_[camera_name];
    ret = trans_wrapper->GetSensor2worldTrans(timestamp, &affine3d_trans);
    pose_matrix = affine3d_trans.matrix();
    if (!ret) {
      pose->ClearCameraPose(camera_name);
      AERROR << "get pose from tf failed, camera_name: " << camera_name;
    } else {
      pose->c2w_poses_[camera_name] = pose_matrix;
      state += 1;
    }
  }
  return state > 0;
}

bool TrafficLightsPerceptionComponent::GetPoseFromTF(
    const double timestamp, const std::string& frame_id,
    const std::string& child_frame_id, Eigen::Matrix4d* pose_matrix) {
  PERCEPTION_PERF_FUNCTION();
  apollo::cyber::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time,
                                 static_cast<float>(tf2_timeout_second_),
                                 &err_string)) {
    AERROR << "Can not find transform. " << timestamp
           << " frame_id: " << frame_id << " child_frame_id: " << child_frame_id
           << " Error info: " << err_string;
    return false;
  }
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform =
        tf2_buffer_->lookupTransform(frame_id, child_frame_id, query_time);
    Eigen::Translation3d translation(
        stamped_transform.transform().translation().x(),
        stamped_transform.transform().translation().y(),
        stamped_transform.transform().translation().z());
    Eigen::Quaterniond rotation(stamped_transform.transform().rotation().qw(),
                                stamped_transform.transform().rotation().qx(),
                                stamped_transform.transform().rotation().qy(),
                                stamped_transform.transform().rotation().qz());
    *pose_matrix = (translation * rotation).matrix();
    ADEBUG << "get pose: " << *pose_matrix;
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  return true;
}

bool TrafficLightsPerceptionComponent::TransformOutputMessage(
    camera::CameraFrame* frame, const std::string& camera_name,
    std::shared_ptr<TrafficLightDetection>* out_msg) {
  PERCEPTION_PERF_FUNCTION();
  const std::map<std::string, TLCamID> CAMERA_ID_TO_TLCAMERA_ID = {
      {"front_24mm", TrafficLightDetection::CAMERA_FRONT_LONG},
      {"front_12mm", TrafficLightDetection::CAMERA_FRONT_NARROW},
      {"front_6mm", TrafficLightDetection::CAMERA_FRONT_SHORT},
      {"front_fisheye", TrafficLightDetection::CAMERA_FRONT_WIDE}};

  auto& lights = frame->traffic_lights;
  auto* header = (*out_msg)->mutable_header();
  double publish_time = apollo::common::time::Clock::NowInSeconds();
  header->set_timestamp_sec(publish_time);  // message publishing time
  AINFO << "set header time sec:" << frame->timestamp;

  // Set traffic light color to unknown before the process
  detected_trafficlight_color_ = base::TLColor::TL_UNKNOWN_COLOR;

  // sec -> nano-sec
  uint64_t ts_int64 = static_cast<uint64_t>(frame->timestamp * 1e9);
  header->set_camera_timestamp(ts_int64);

  if (CAMERA_ID_TO_TLCAMERA_ID.find(camera_name) ==
      CAMERA_ID_TO_TLCAMERA_ID.end()) {
    AERROR << "unknown camera_name: " << camera_name;
    return false;
  }
  (*out_msg)->set_camera_id(CAMERA_ID_TO_TLCAMERA_ID.at(camera_name));

  // Do voting from multiple traffic light detections
  cnt_r_ = 0;
  int max_r_id = -1;
  double max_r_conf = 0;

  cnt_g_ = 0;
  int max_g_id = -1;
  double max_g_conf = 0;

  cnt_y_ = 0;
  int max_y_id = -1;
  double max_y_conf = 0;

  cnt_u_ = 0;

  int max_n_id = -1;

  for (int i = 0; i < static_cast<int>(lights.size()); i++) {
    switch (lights.at(i)->status.color) {
      case base::TLColor::TL_RED:
        // quick fix for 0 confidence color decision
        if (std::abs(lights.at(i)->status.confidence) <
            std::numeric_limits<double>::min()) {
          lights.at(i)->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_n_id = i;
          break;
        }
        cnt_r_ += lights.at(i)->status.confidence;
        if (lights.at(i)->status.confidence >= max_r_conf) {
          max_r_id = i;
          max_r_conf = lights.at(i)->status.confidence;
        }
        break;
      case base::TLColor::TL_GREEN:
        // quick fix for 0 confidence color decision
        if (std::abs(lights.at(i)->status.confidence) <
            std::numeric_limits<double>::min()) {
          lights.at(i)->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_n_id = i;
          break;
        }
        cnt_g_ += lights.at(i)->status.confidence;
        if (lights.at(i)->status.confidence >= max_g_conf) {
          max_g_id = i;
          max_g_conf = lights.at(i)->status.confidence;
        }
        break;
      case base::TLColor::TL_YELLOW:
        // quick fix for 0 confidence color decision
        if (std::abs(lights.at(i)->status.confidence) <
            std::numeric_limits<double>::min()) {
          lights.at(i)->status.color = base::TLColor::TL_UNKNOWN_COLOR;
          max_n_id = i;
          break;
        }
        cnt_y_ += lights.at(i)->status.confidence;
        if (lights.at(i)->status.confidence >= max_y_conf) {
          max_y_id = i;
          max_y_conf = lights.at(i)->status.confidence;
        }
        break;
      case base::TLColor::TL_UNKNOWN_COLOR:
        cnt_u_ += lights.at(i)->status.confidence;
        max_n_id = i;
        break;
      default:
        max_n_id = i;
        break;
    }
  }

  int max_light_id = -1;
  if (cnt_r_ >= cnt_g_ && cnt_r_ >= cnt_y_ && cnt_r_ > 0) {
    max_light_id = max_r_id;
  } else if (cnt_y_ > cnt_r_ && cnt_y_ >= cnt_g_) {
    max_light_id = max_y_id;
  } else if (cnt_g_ > cnt_r_ && cnt_g_ > cnt_y_) {
    max_light_id = max_g_id;
  } else if (cnt_r_ == 0 && cnt_g_ == 0 && cnt_y_ == 0) {
    max_light_id = max_n_id;
  }

  // swap the final output light to the first place
  if (max_light_id > 0) {
    std::swap(lights[0], lights[max_light_id]);
  }

  if (max_light_id >= 0) {
    for (size_t i = 0; i < lights.size(); i++) {
      apollo::perception::TrafficLight* light_result =
          (*out_msg)->add_traffic_light();
      light_result->set_id(lights.at(i)->id);
      light_result->set_confidence(lights.at(0)->status.confidence);
      light_result->set_color(
          static_cast<apollo::perception::TrafficLight_Color>(
              lights.at(0)->status.color));
      light_result->set_blink(lights.at(0)->status.blink);
    }
    // set contain_lights
    (*out_msg)->set_contain_lights(lights.size() > 0);
    detected_trafficlight_color_ = lights.at(0)->status.color;
  }
  // add traffic light debug info
  if (!TransformDebugMessage(frame, out_msg)) {
    AERROR << "ProcComponent::Proc failed to transform debug msg.";
    return false;
  }

  return true;
}

void TrafficLightsPerceptionComponent::TransRect2Box(
    const base::RectI& rect, apollo::perception::TrafficLightBox* box) {
  box->set_x(rect.x);
  box->set_y(rect.y);
  box->set_width(rect.width);
  box->set_height(rect.height);
}

double TrafficLightsPerceptionComponent::stopline_distance(
    const Eigen::Matrix4d& cam_pose) {
  if (stoplines_.empty()) {
    AWARN << "compute car to stopline's distance failed(no stopline). "
          << "cam_pose:" << cam_pose;
    return -1;
  }
  const apollo::hdmap::Curve& stopline = stoplines_.Get(0);
  if (stopline.segment().empty()) {
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

  if (stopline.segment(0).line_segment().point().empty()) {
    AWARN << "compute car to stopline's distance "
          << "failed(stopline has no point). "
          << "cam_pose:" << cam_pose
          << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  Eigen::Vector3d stopline_pt(stopline.segment(0).line_segment().point(0).x(),
                              stopline.segment(0).line_segment().point(0).y(),
                              stopline.segment(0).line_segment().point(0).z());
  Eigen::Vector3d stopline_pt_cam =
      (cam_pose.inverse() *
       Eigen::Vector4d(stopline_pt(0), stopline_pt(1), stopline_pt(2), 1.0))
          .head(3);

  return stopline_pt_cam(2);
}

bool TrafficLightsPerceptionComponent::TransformDebugMessage(
    const camera::CameraFrame* frame,
    std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg) {
  PERCEPTION_PERF_FUNCTION();
  const auto& lights = frame->traffic_lights;
  // add traffic light debug info
  TrafficLightDebug* light_debug = (*out_msg)->mutable_traffic_light_debug();

  // signal number
  light_debug->set_signal_num(static_cast<int>(lights.size()));

  if (!lights.empty() && !lights[0]->region.debug_roi.empty()) {
    const auto& debug_roi = lights[0]->region.debug_roi;
    // Crop ROI
    TransRect2Box(debug_roi[0], light_debug->mutable_cropbox());

    // debug ROI (candidate detection boxes)
    for (auto iter = debug_roi.begin() + 1; iter != debug_roi.end(); ++iter) {
      TransRect2Box(*iter, light_debug->add_box());
      TransRect2Box(*iter, light_debug->add_debug_roi());
    }
  }

  for (const auto& light : lights) {
    // Detection ROI
    auto* box = light_debug->add_box();
    TransRect2Box(light->region.detection_roi, box);
    box->set_color(static_cast<TrafficLight_Color>(light->status.color));
    box->set_selected(true);

    // Projection ROI
    TransRect2Box(light->region.projection_roi, light_debug->add_box());
    TransRect2Box(light->region.projection_roi,
                  light_debug->add_projected_roi());

    // Crop ROI
    TransRect2Box(light->region.debug_roi[0], light_debug->add_crop_roi());

    // Rectified ROI
    auto* rectified_roi = light_debug->add_rectified_roi();
    TransRect2Box(light->region.detection_roi, rectified_roi);
    rectified_roi->set_color(
        static_cast<TrafficLight_Color>(light->status.color));
    rectified_roi->set_selected(true);
  }

  if (lights.size() > 0) {
    camera::CarPose pose;
    if (GetCarPose(frame->timestamp, &pose)) {
      Eigen::Matrix4d cam_pose;
      cam_pose = pose.c2w_poses_.at("front_6mm");
      light_debug->set_distance_to_stop_line(stopline_distance(cam_pose));
    } else {
      AERROR << "error occurred in calc distance to stop line";
    }
  }

  if (FLAGS_start_visualizer) {
    Visualize(*frame, lights);
  }

  return true;
}

void TrafficLightsPerceptionComponent::Visualize(
    const camera::CameraFrame& frame,
    const std::vector<base::TrafficLightPtr>& lights) const {
  char str[100];
  std::string tl_string;
  cv::Scalar tl_color;

  if (lights.empty()) {
    return;
  }
  cv::Mat output_image(image_height_, image_width_, CV_8UC3,
                       cv::Scalar(0, 0, 0));
  base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  frame.data_provider->GetImage(image_options, &out_image);
  memcpy(output_image.data, out_image.cpu_data(),
         out_image.total() * sizeof(uint8_t));

  for (const auto& light : lights) {
    // Crop ROI
    const auto& crop_roi = light->region.debug_roi[0];
    const cv::Rect rect_crop(crop_roi.x, crop_roi.y, crop_roi.width,
                             crop_roi.height);
    if (light == lights[0])
      cv::rectangle(output_image, rect_crop, cv::Scalar(255, 255, 255), 2);
    else
      cv::rectangle(output_image, rect_crop, cv::Scalar(255, 255, 255));

    // Project lights
    const auto& projection_roi = light->region.projection_roi;
    const cv::Rect projection_rect(projection_roi.x, projection_roi.y,
                                   projection_roi.width, projection_roi.height);
    cv::rectangle(output_image, projection_rect, cv::Scalar(255, 0, 0), 3);

    // Detect lights
    const auto& rectified_roi = light->region.detection_roi;
    const cv::Rect rectified_rect(rectified_roi.x, rectified_roi.y,
                                  rectified_roi.width, rectified_roi.height);
    cv::Scalar tl_color;
    std::map<base::TLColor, TLInfo>::iterator itor =
        s_tl_infos.find(light->status.color);
    if (itor != s_tl_infos.end()) {
      tl_color = itor->second.tl_color_;
      tl_string = itor->second.tl_string_;
    } else {
      tl_color = cv::Scalar(255, 255, 255);
      tl_string = "UNKNOWN";
    }
    snprintf(str, sizeof(str), "ID:%s C:%.3lf", light->id.c_str(),
             light->status.confidence);
    cv::rectangle(output_image, rectified_rect, tl_color, 2);
    cv::putText(output_image, str,
                cv::Point(rectified_roi.x + 30,
                          rectified_roi.y + rectified_roi.height + 30),
                cv::FONT_HERSHEY_DUPLEX, 1.0, tl_color, 2);
  }

  // Show text of voting results
  std::map<base::TLColor, TLInfo>::iterator itor =
      s_tl_infos.find(detected_trafficlight_color_);
  if (itor != s_tl_infos.end()) {
    tl_color = itor->second.tl_color_;
    tl_string = itor->second.tl_string_ex_;
  } else {
    tl_color = cv::Scalar(255, 255, 255);
    tl_string = "UNKNOWN traffic light";
  }
  double all = cnt_r_ + cnt_g_ + cnt_y_ + cnt_u_;
  if (all < 0.0001) {
    all = 1.0;
  }
  cv::putText(output_image, tl_string, cv::Point(10, 90),
              cv::FONT_HERSHEY_DUPLEX, 2.0, tl_color, 3);

  snprintf(str, sizeof(str), "Red lights:%.2f", cnt_r_ / all);
  cv::putText(output_image, str, cv::Point(10, 150), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 0, 255), 3);
  snprintf(str, sizeof(str), "Green lights:%.2f", cnt_g_ / all);
  cv::putText(output_image, str, cv::Point(10, 200), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 255, 0), 3);
  snprintf(str, sizeof(str), "Yellow lights:%.2f", cnt_y_ / all);
  cv::putText(output_image, str, cv::Point(10, 250), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(0, 255, 255), 3);
  snprintf(str, sizeof(str), "Unknown lights:%.2f", cnt_u_ / all);
  cv::putText(output_image, str, cv::Point(10, 300), cv::FONT_HERSHEY_DUPLEX,
              1.5, cv::Scalar(255, 255, 255), 3);

  cv::resize(output_image, output_image, cv::Size(), 0.5, 0.5);
  cv::imshow("Traffic Light", output_image);
  cv::imwrite(absl::StrCat("/apollo/debug_vis/", frame.timestamp, ".jpg"),
              output_image);
  cvWaitKey(30);
}

void TrafficLightsPerceptionComponent::SyncV2XTrafficLights(
    camera::CameraFrame* frame) {
  const double camera_frame_timestamp = frame->timestamp;
  auto sync_single_light = [&](base::TrafficLightPtr light) {
    for (auto itr = v2x_msg_buffer_.rbegin(); itr != v2x_msg_buffer_.rend();
         ++itr) {
      double v2x_timestamp = (*itr).header().timestamp_sec();
      // find close enough v2x msg
      if (std::fabs(camera_frame_timestamp - v2x_timestamp) <
          v2x_sync_interval_seconds_) {
        const int v2x_lights_num =
            (*itr).current_lane_trafficlight().single_traffic_light_size();
        const auto& v2x_lights = (*itr).current_lane_trafficlight();
        for (int i = 0; i < v2x_lights_num; ++i) {
          const auto& v2x_light = v2x_lights.single_traffic_light(i);
          // check signal id
          if (light->id != v2x_light.id()) {
            continue;
          }
          base::TLColor v2x_color = base::TLColor::TL_UNKNOWN_COLOR;
          bool blink = false;
          switch (v2x_light.color()) {
            default:
            case apollo::v2x::SingleTrafficLight::UNKNOWN:
              v2x_color = base::TLColor::TL_UNKNOWN_COLOR;
              break;
            case apollo::v2x::SingleTrafficLight::RED:
              v2x_color = base::TLColor::TL_RED;
              break;
            case apollo::v2x::SingleTrafficLight::YELLOW:
              v2x_color = base::TLColor::TL_YELLOW;
              break;
            case apollo::v2x::SingleTrafficLight::GREEN:
              v2x_color = base::TLColor::TL_GREEN;
              break;
            case apollo::v2x::SingleTrafficLight::BLACK:
              v2x_color = base::TLColor::TL_BLACK;
              break;
            case apollo::v2x::SingleTrafficLight::FLASH_GREEN:
              v2x_color = base::TLColor::TL_GREEN;
              blink = true;
              break;
          }
          // use v2x result directly
          AINFO << "Sync V2X success. update color from "
                << static_cast<int>(light->status.color) << " to "
                << static_cast<int>(v2x_color) << "; signal id: " << light->id;
          light->status.color = v2x_color;
          light->status.blink = blink;
        }
        break;
      }
    }
  };
  for (auto& light : frame->traffic_lights) {
    sync_single_light(light);
  }
}

void TrafficLightsPerceptionComponent::SendSimulationMsg() {
  auto out_msg = std::make_shared<TrafficLightDetection>();
  writer_->Write(out_msg);
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
