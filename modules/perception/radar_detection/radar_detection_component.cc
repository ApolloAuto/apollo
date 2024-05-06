/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/radar_detection/radar_detection_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/onboard/common_flags/common_flags.h"

using Clock = apollo::cyber::Clock;

namespace apollo {
namespace perception {
namespace radar {

bool RadarDetectionComponent::Init() {
  RadarComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Radar component config: " << comp_config.DebugString();

  // To load component configs
  tf_child_frame_id_ = comp_config.tf_child_frame_id();
  radar_forward_distance_ = comp_config.radar_forward_distance();
  odometry_channel_name_ = comp_config.odometry_channel_name();

  if (!algorithm::SensorManager::Instance()->GetSensorInfo(
          comp_config.radar_name(), &radar_info_)) {
    AERROR << "Failed to get sensor info, sensor name: "
           << comp_config.radar_name();
    return false;
  }

  writer_ = node_->CreateWriter<onboard::SensorFrameMessage>(
      comp_config.output_channel_name());

  // Init algorithm plugin
  ACHECK(InitAlgorithmPlugin(comp_config))
      << "Failed to init algorithm plugin.";

  radar2world_trans_.Init(tf_child_frame_id_);
  radar2novatel_trans_.Init(tf_child_frame_id_);
  localization_subscriber_.Init(
      odometry_channel_name_,
      odometry_channel_name_ + '_' + comp_config.radar_name());
  return true;
}

bool RadarDetectionComponent::Proc(const std::shared_ptr<ContiRadar>& message) {
  AINFO << "Enter radar preprocess, message timestamp: "
        << message->header().timestamp_sec() << " current timestamp "
        << Clock::NowInSeconds();
  auto out_message = std::make_shared<onboard::SensorFrameMessage>();

  if (!InternalProc(message, out_message)) {
    return false;
  }
  writer_->Write(out_message);
  return true;
}

bool RadarDetectionComponent::InitAlgorithmPlugin(
    const RadarComponentConfig& config) {
  if (onboard::FLAGS_obs_enable_hdmap_input) {
    hdmap_input_ = map::HDMapInput::Instance();
    ACHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";
  }

  auto preprocessor_param = config.preprocessor_param();
  PreprocessorInitOptions preprocessor_init_options;
  preprocessor_init_options.config_path = preprocessor_param.config_path();
  preprocessor_init_options.config_file = preprocessor_param.config_file();
  BasePreprocessor* radar_preprocessor =
      BasePreprocessorRegisterer::GetInstanceByName(preprocessor_param.name());
  CHECK_NOTNULL(radar_preprocessor);
  radar_preprocessor_.reset(radar_preprocessor);
  ACHECK(radar_preprocessor_->Init(preprocessor_init_options))
      << "Failed to init radar preprocessor.";

  auto perception_param = config.perception_param();
  PerceptionInitOptions perception_init_options;
  perception_init_options.config_path = perception_param.config_path();
  perception_init_options.config_file = perception_param.config_file();
  BaseRadarObstaclePerception* radar_perception =
      BaseRadarObstaclePerceptionRegisterer::GetInstanceByName(
          perception_param.name());
  CHECK_NOTNULL(radar_perception);
  radar_perception_.reset(radar_perception);
  ACHECK(radar_perception_->Init(perception_init_options))
      << "Failed to init radar perception.";
  return true;
}

bool RadarDetectionComponent::InternalProc(
    const std::shared_ptr<ContiRadar>& in_message,
    std::shared_ptr<onboard::SensorFrameMessage> out_message) {
  PERF_FUNCTION_WITH_INDICATOR(radar_info_.name);
  ContiRadar raw_obstacles = *in_message;
  {
    std::unique_lock<std::mutex> lock(_mutex);
    ++seq_num_;
  }
  double timestamp = in_message->header().timestamp_sec();
  PERF_BLOCK_START();
  // Init preprocessor_options
  PreprocessorOptions preprocessor_options;
  ContiRadar corrected_obstacles;
  radar_preprocessor_->Preprocess(raw_obstacles, preprocessor_options,
                                  &corrected_obstacles);
  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "radar_preprocessor");
  timestamp = corrected_obstacles.header().timestamp_sec();

  out_message->timestamp_ = timestamp;
  out_message->seq_num_ = seq_num_;
  out_message->process_stage_ =
      onboard::ProcessStage::LONG_RANGE_RADAR_DETECTION;
  out_message->sensor_id_ = radar_info_.name;

  // Init radar perception options
  RadarPerceptionOptions options;
  options.sensor_name = radar_info_.name;
  // Init detector_options
  Eigen::Affine3d radar_trans;
  if (!radar2world_trans_.GetSensor2worldTrans(timestamp, &radar_trans)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: " << timestamp;
    return true;
  }
  Eigen::Affine3d radar2novatel_trans;
  if (!radar2novatel_trans_.GetTrans(timestamp, &radar2novatel_trans, "novatel",
                                     tf_child_frame_id_)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get radar2novatel trans at time: " << timestamp;
    return true;
  }
  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetSensor2worldTrans");
  Eigen::Matrix4d radar2world_pose = radar_trans.matrix();
  options.detector_options.radar2world_pose = &radar2world_pose;
  Eigen::Matrix4d radar2novatel_trans_m = radar2novatel_trans.matrix();
  options.detector_options.radar2novatel_trans = &radar2novatel_trans_m;
  if (!GetCarLocalizationSpeed(timestamp,
                               &(options.detector_options.car_linear_speed),
                               &(options.detector_options.car_angular_speed))) {
    AERROR << "Failed to call get_car_speed. [timestamp: " << timestamp;
    // return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetCarSpeed");
  // Init roi_filter_options
  base::PointD position;
  position.x = radar_trans(0, 3);
  position.y = radar_trans(1, 3);
  position.z = radar_trans(2, 3);
  options.roi_filter_options.roi.reset(new base::HdmapStruct());
  if (onboard::FLAGS_obs_enable_hdmap_input) {
    hdmap_input_->GetRoiHDMapStruct(position, radar_forward_distance_,
                                    options.roi_filter_options.roi);
  }
  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetRoiHDMapStruct");
  // Radar perception
  std::vector<base::ObjectPtr> radar_objects;
  if (!radar_perception_->Perceive(corrected_obstacles, options,
                                   &radar_objects)) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "RadarDetector Proc failed.";
    return true;
  }
  out_message->frame_.reset(new base::Frame());
  out_message->frame_->sensor_info = radar_info_;
  out_message->frame_->timestamp = timestamp;
  out_message->frame_->sensor2world_pose = radar_trans;
  out_message->frame_->objects = radar_objects;

  PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "radar_perception");
  return true;
}

bool RadarDetectionComponent::GetCarLocalizationSpeed(
    double timestamp, Eigen::Vector3f* car_linear_speed,
    Eigen::Vector3f* car_angular_speed) {
  if (car_linear_speed == nullptr) {
    AERROR << "car_linear_speed is not available";
    return false;
  }
  (*car_linear_speed) = Eigen::Vector3f::Zero();

  if (car_angular_speed == nullptr) {
    AERROR << "car_angular_speed is not available";
    return false;
  }
  (*car_angular_speed) = Eigen::Vector3f::Zero();
  std::shared_ptr<LocalizationEstimate const> loct_ptr;
  if (!localization_subscriber_.LookupNearest(timestamp, &loct_ptr)) {
    AERROR << "Cannot get car speed.";
    return false;
  }
  (*car_linear_speed)[0] =
      static_cast<float>(loct_ptr->pose().linear_velocity().x());
  (*car_linear_speed)[1] =
      static_cast<float>(loct_ptr->pose().linear_velocity().y());
  (*car_linear_speed)[2] =
      static_cast<float>(loct_ptr->pose().linear_velocity().z());
  (*car_angular_speed)[0] =
      static_cast<float>(loct_ptr->pose().angular_velocity().x());
  (*car_angular_speed)[1] =
      static_cast<float>(loct_ptr->pose().angular_velocity().y());
  (*car_angular_speed)[2] =
      static_cast<float>(loct_ptr->pose().angular_velocity().z());

  return true;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
