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
#include "modules/perception/traffic_light_region_proposal/traffic_light_region_proposal_component.h"

#include <sys/stat.h>
#include <unistd.h>

#include <limits>
#include <map>
#include <utility>

#include <boost/algorithm/string.hpp>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/camera/common/data_provider.h"
#include "modules/perception/common/onboard/common_flags/common_flags.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::cyber::Clock;
using apollo::cyber::common::GetAbsolutePath;
using apollo::perception::algorithm::SensorManager;

bool TrafficLightsPerceptionComponent::Init() {
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

  return true;
}

int TrafficLightsPerceptionComponent::InitConfig() {
  apollo::perception::trafficlight::TrafficLight traffic_light_param;
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

  gpu_id_ = traffic_light_param.gpu_id();
  proposal_output_channel_name_ =
      traffic_light_param.proposal_output_channel_name();
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

  tl_preprocessor_name_ = traffic_light_param.plugin_param().name();

  default_image_border_size_ = traffic_light_param.default_image_border_size();

  traffic_detect_writer_ =
      node_->CreateWriter<TrafficDetectMessage>(proposal_output_channel_name_);

  return cyber::SUCC;
}

int TrafficLightsPerceptionComponent::InitAlgorithmPlugin() {
  // init preprocessor
  trafficlight::BaseTLPreprocessor* preprocessor =
      trafficlight::BaseTLPreprocessorRegisterer::GetInstanceByName(
          tl_preprocessor_name_);
  CHECK_NOTNULL(preprocessor);
  preprocessor_.reset(preprocessor);

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

int TrafficLightsPerceptionComponent::InitCameraFrame() {
  data_provider_init_options_.image_height = image_height_;
  data_provider_init_options_.image_width = image_width_;

  data_provider_init_options_.device_id = gpu_id_;
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
  PERF_FUNCTION()
  std::lock_guard<std::mutex> lck(mutex_);
  double receive_img_timestamp = Clock::NowInSeconds();
  double image_msg_ts = msg->measurement_time();
  image_msg_ts += image_timestamp_offset_;
  last_sub_camera_image_ts_[camera_name] = image_msg_ts;

  if (!CheckCameraImageStatus(image_msg_ts, check_image_status_interval_thresh_,
                              camera_name)) {
    AERROR << "CheckCameraImageStatus failed";
    return;
  }
  // trafficlight preprocess msg
  std::shared_ptr<TrafficDetectMessage> preprocess_message(
      new (std::nothrow) TrafficDetectMessage);
  preprocess_message->timestamp_ = image_msg_ts;

  auto& frame = preprocess_message->traffic_light_frame_;
  frame.reset(new camera::TrafficLightFrame);
  frame->timestamp = image_msg_ts;

  trafficlight::TLPreprocessorOption preprocess_option;
  preprocess_option.image_borders_size = &image_border_sizes_;

  // query pose and signals, add cached camera selection by lights' projections
  if (!UpdateCameraSelection(image_msg_ts, preprocess_option, frame)) {
    AWARN << "add_cached_camera_selection failed, ts: " << image_msg_ts;
  }

  // skipping frame according to last proc image timestamp
  if (last_proc_image_ts_ > 0.0 &&
      receive_img_timestamp - last_proc_image_ts_ < proc_interval_seconds_) {
    AINFO << "skip current image, img_ts: " << FORMAT_TIMESTAMP(image_msg_ts)
          << " , receive_img_timestamp: "
          << FORMAT_TIMESTAMP(receive_img_timestamp)
          << " ,_last_proc_image_ts: " << FORMAT_TIMESTAMP(last_proc_image_ts_)
          << " , _proc_interval_seconds: "
          << FORMAT_TIMESTAMP(proc_interval_seconds_);
    return;
  }
  // sync image with cached projections
  bool sync_image_ok =
      preprocessor_->SyncInformation(image_msg_ts, camera_name);

  if (!sync_image_ok) {
    AINFO << "PreprocessComponent not publish image, ts:" << image_msg_ts
          << ", camera_name: " << camera_name;
    //    SendSimulationMsg();
    return;
  }

  // Fill camera frame
  camera::DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  frame->data_provider = data_providers_map_.at(camera_name);
  frame->data_provider->FillImageData(
      image_height_, image_width_,
      reinterpret_cast<const uint8_t*>(msg->data().data()), msg->encoding());
  frame->timestamp = image_msg_ts;
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
                              frame, preprocess_message)) {
    AINFO << "VerifyLightsProjection on image failed, ts: " << image_msg_ts
          << ", camera_name: " << camera_name
          << " last_query_tf_ts_: " << last_query_tf_ts_
          << " need update_camera_selection immediately,"
          << " reset last_query_tf_ts_ to -1";
    last_query_tf_ts_ = -1.0;
  }
  last_proc_image_ts_ = Clock::NowInSeconds();

  preprocess_message->stoplines_ = stoplines_;

  bool send_message_ret = traffic_detect_writer_->Write(preprocess_message);
  AINFO << "send out preprocess msg, ts: " << image_msg_ts
        << "ret: " << send_message_ret;
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

    light->semantic = cur_semantic;
    traffic_lights->push_back(light);
    stoplines_ = signal.stop_line();
  }
}

bool TrafficLightsPerceptionComponent::QueryPoseAndSignals(
    const double ts, camera::CarPose* pose,
    std::vector<apollo::hdmap::Signal>* signals) {
  // get pose
  if (!GetCarPose(ts, pose)) {
    AINFO << "query_pose_and_signals failed to get car pose, ts:" << ts;
    return false;
  }

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
    const double& ts, const trafficlight::TLPreprocessorOption& option,
    const std::string& camera_name,
    std::shared_ptr<camera::TrafficLightFrame> frame,
    std::shared_ptr<TrafficDetectMessage> msg) {
  camera::CarPose* pose_ptr = new camera::CarPose();
  auto& pose = *pose_ptr;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(ts, pose_ptr, &signals)) {
    AERROR << "query_pose_and_signals failed, ts: " << ts;
    return false;
  }
  auto& carpose_ptr = msg->carpose_;
  carpose_ptr.reset(pose_ptr);

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
    double timestamp, const trafficlight::TLPreprocessorOption& option,
    std::shared_ptr<camera::TrafficLightFrame> frame) {
  const double current_ts = Clock::NowInSeconds();
  if (last_query_tf_ts_ > 0.0 &&
      current_ts - last_query_tf_ts_ < query_tf_interval_seconds_) {
    AINFO << "skip current tf msg, img_ts: " << FORMAT_TIMESTAMP(timestamp)
          << " , last_query_tf_ts_: " << FORMAT_TIMESTAMP(last_query_tf_ts_);
    return true;
  }
  AINFO << "start select camera";

  camera::CarPose pose;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(timestamp, &pose, &signals)) {
    AINFO << "query_pose_and_signals failed, ts: "
          << FORMAT_TIMESTAMP(timestamp);
    return false;
  }
  last_query_tf_ts_ = current_ts;

  GenerateTrafficLights(signals, &frame->traffic_lights);
  AINFO << "hd map signals " << frame->traffic_lights.size();

  if (!preprocessor_->UpdateCameraSelection(pose, option,
                                            &frame->traffic_lights)) {
    AERROR << "add_cached_lights_projections failed, ts: "
           << FORMAT_TIMESTAMP(timestamp);
  } else {
    AINFO << "add_cached_lights_projections succeed, ts: "
          << FORMAT_TIMESTAMP(timestamp);
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
  bool camera_ok = true;
  std::string no_image_camera_names = "";
  for (const auto& pr : last_sub_camera_image_ts_) {
    const auto cam_name = pr.first;
    double last_sub_camera_ts = pr.second;
    // should be 0.0, change to 1 in case of float precision
    if (last_sub_camera_ts < 1.0 || timestamp - last_sub_camera_ts > interval) {
      preprocessor_->SetCameraWorkingFlag(cam_name, false);
      AWARN << "camera is probably not working"
            << " , current ts: " << FORMAT_TIMESTAMP(timestamp)
            << " , last_sub_camera_ts: " << FORMAT_TIMESTAMP(last_sub_camera_ts)
            << " , camera_name: " << cam_name;
      camera_ok = false;
      AINFO << "camera status:" << camera_ok;
      no_image_camera_names += (" " + cam_name);
    }
  }

  bool is_camera_working = false;
  if (!preprocessor_->GetCameraWorkingFlag(camera_name, &is_camera_working)) {
    AERROR << "get_camera_is_working_flag ts: " << FORMAT_TIMESTAMP(timestamp)
           << " camera_name: " << camera_name;
    return false;
  }

  if (!is_camera_working) {
    if (!preprocessor_->SetCameraWorkingFlag(camera_name, true)) {
      AERROR << "set_camera_is_working_flag ts: " << FORMAT_TIMESTAMP(timestamp)
             << " camera_name: " << camera_name;
      return false;
    }
  }
  return true;
}

bool TrafficLightsPerceptionComponent::GetCarPose(const double timestamp,
                                                  camera::CarPose* pose) {
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
  apollo::cyber::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time,
                                 static_cast<float>(tf2_timeout_second_),
                                 &err_string)) {
    AERROR << "Can not find transform. " << FORMAT_TIMESTAMP(timestamp)
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

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
