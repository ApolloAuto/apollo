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
#include "modules/perception/onboard/component/radar_detection_component.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/utils/perf.h"

namespace apollo {
namespace perception {
namespace onboard {

DEFINE_string(obs_localization_channel, "/localization/100hz/localization_pose",
              "subscribe localization channel name");
DEFINE_string(obs_radar_obstacle_perception, "RadarObstaclePerception",
              "radar obstacle perception");
DEFINE_bool(obs_convert2gps_timestamp, false,
            "whether convert timestamp to gps timestamp.");

int RadarDetectionComponent::Init() {
  std::string radar_name;
  READ_CONF(this->_name, "radar_name", radar_name);
  READ_CONF(this->_name, "tf_child_frame_id", tf_child_frame_id_);
  READ_CONF(this->_name, "radar_forward_distance", radar_forward_distance_);
  READ_CONF(this->_name, "radar_preprocessor", preprocessor_name_);
  READ_CONF(this->_name, "radar_pipeline", pipeline_name_);

  common::SensorManager* sensor_manager =
      lib::Singleton<common::SensorManager>::get_instance();
  if (sensor_manager == nullptr) {
    LOG_ERROR << "Failed to get sensor manager instance";
    return FAIL;
  }
  bool ret = sensor_manager->GetSensorInfo(radar_name, &radar_info_);
  if (!ret) {
    LOG_ERROR << "Failed to get sensor info, sensor name: " << radar_name;
    return FAIL;
  }

  // init algorithm plugin
  CHECK(InitAlgorithmPlugin() == SUCC) << "Failed to init algorithm plugin.";
  radar2world_trans_.Init(tf_child_frame_id_);
  radar2novatel_trans_.Init(tf_child_frame_id_);
  localization_subscriber_.Init(
      FLAGS_obs_localization_channel,
      FLAGS_obs_localization_channel + '_' + radar_name);
  return SUCC;
}

int RadarDetectionComponent::Proc(
    const std::shared_ptr<RadarObsArray const>& message) {
  LOG_INFO << "Enter radar preprocess, message timestamp: "
           << GLOG_TIMESTAMP(message->measurement_time())
           << " current timestamp " << lib::TimeUtil::GetCurrentTime();
  std::shared_ptr<SensorFrameMessage> out_message(new (std::nothrow)
                                                      SensorFrameMessage);
  int status = InternalProc(message, out_message);
  if (status == SUCC) {
    Send("/perception/inner/PrefusedObjects", out_message);
    LOG_INFO << "Send radar processing output message.";
  }
  return status;

  return 1;
}

int RadarDetectionComponent::InitAlgorithmPlugin() {
  LOG_INFO << "onboard radar_preprocessor: " << preprocessor_name_;
  if (FLAGS_obs_enable_hdmap_input) {
    hdmap_input_ = lib::Singleton<map::HDMapInput>::get_instance();
    CHECK_NOTNULL(hdmap_input_);
    CHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";
  }
  radar::BasePreprocessor* preprocessor =
      radar::BasePreprocessorRegisterer::GetInstanceByName(preprocessor_name_);
  CHECK_NOTNULL(preprocessor);
  radar_preprocessor_.reset(preprocessor);
  CHECK(radar_preprocessor_->Init()) << "Failed to init radar preprocessor.";
  radar::BaseRadarObstaclePerception* radar_perception =
      radar::BaseRadarObstaclePerceptionRegisterer::GetInstanceByName(
          FLAGS_obs_radar_obstacle_perception);
  CHECK(radar_perception != nullptr) << "No radar obstacle perception named "
                                     << FLAGS_obs_radar_obstacle_perception;
  radar_perception_.reset(radar_perception);
  CHECK(radar_perception_->Init(pipeline_name_))
      << "Failed to init radar perception.";
  LOG_INFO << "Init algorithm plugin successfully.";
  return SUCC;
}

int RadarDetectionComponent::InternalProc(
    const std::shared_ptr<RadarObsArray const>& in_message,
    std::shared_ptr<SensorFrameMessage> out_message) {
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(radar_info_.name);
  RadarObsArray raw_obstacles = *in_message;
  {
    std::unique_lock<std::mutex> lock(_mutex);
    ++seq_num_;
  }
  double timestamp = in_message->measurement_time();
  const double cur_time = lib::TimeUtil::GetCurrentTime();
  const double start_latency = (cur_time - timestamp) * 1e3;
  LOG_INFO << "FRAME_STATISTICS:Radar:Start:msg_time["
           << GLOG_TIMESTAMP(timestamp) << "]:cur_time["
           << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
           << "]";
  PERCEPTION_PERF_BLOCK_START();
  // init preprocessor_options
  radar::PreprocessorOptions preprocessor_options;
  RadarObsArray corrected_obstacles;
  radar_preprocessor_->Preprocess(raw_obstacles, preprocessor_options,
                                  &corrected_obstacles);
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name,
                                           "radar_preprocessor");
  timestamp = corrected_obstacles.measurement_time();
  if (FLAGS_obs_convert2gps_timestamp) {
    timestamp = lib::TimeUtil::Unix2Gps(timestamp);
  }

  out_message->timestamp_ = timestamp;
  out_message->seq_num_ = seq_num_;
  out_message->process_stage_ = ProcessStage::LONG_RANGE_RADAR_DETECTION;
  out_message->sensor_id_ = radar_info_.name;

  // init radar perception options
  radar::RadarPerceptionOptions options;
  options.sensor_name = radar_info_.name;
  // init detector_options
  Eigen::Affine3d radar_trans;
  if (radar2world_trans_.GetSensor2worldTrans(timestamp, &radar_trans) !=
      SUCC) {
    out_message->error_code_ = ::adu::common::perception::ERROR_TF;
    LOG_ERROR << "Fail to get pose at time: " << timestamp;
    return SUCC;
  }
  Eigen::Affine3d radar2novatel_trans;
  if (radar2novatel_trans_.GetTrans(timestamp, &radar2novatel_trans, "novatel",
                                    tf_child_frame_id_) != SUCC) {
    out_message->error_code_ = ::adu::common::perception::ERROR_TF;
    LOG_ERROR << "Fail to get radar2novatel trans at time: " << timestamp;
    return SUCC;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name,
                                           "GetSensor2worldTrans");
  Eigen::Matrix4d radar2world_pose = radar_trans.matrix();
  options.detector_options.radar2world_pose = &radar2world_pose;
  Eigen::Matrix4d radar2novatel_trans_m = radar2novatel_trans.matrix();
  options.detector_options.radar2novatel_trans = &radar2novatel_trans_m;
  if (GetCarLocalizationSpeed(
          timestamp, &(options.detector_options.car_linear_speed),
          &(options.detector_options.car_angular_speed)) != SUCC) {
    LOG_ERROR << "Failed to call get_car_speed. [timestamp: "
              << GLOG_TIMESTAMP(timestamp);
    return FAIL;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name, "GetCarSpeed");
  // init roi_filter_options
  base::PointD position;
  position.x = radar_trans(0, 3);
  position.y = radar_trans(1, 3);
  position.z = radar_trans(2, 3);
  options.roi_filter_options.roi.reset(new base::HdmapStruct());
  if (FLAGS_obs_enable_hdmap_input) {
    hdmap_input_->GetRoiHDMapStruct(position, radar_forward_distance_,
                                    options.roi_filter_options.roi);
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name,
                                           "GetRoiHDMapStruct");
  // init object_filter_options
  // init track_options
  // init object_builder_options
  std::vector<base::ObjectPtr> radar_objects;
  bool result =
      radar_perception_->Perceive(corrected_obstacles, options, &radar_objects);

  if (!result) {
    out_message->error_code_ = ::adu::common::perception::ERROR_PROCESS;
    LOG_ERROR << "RadarDetector Proc failed.";
    return SUCC;
  }
  out_message->frame_.reset(new base::Frame());
  out_message->frame_->sensor_info = radar_info_;
  out_message->frame_->timestamp = timestamp;
  out_message->frame_->sensor2world_pose = radar_trans;
  out_message->frame_->objects = radar_objects;

  const double end_timestamp = lib::TimeUtil::GetCurrentTime();
  const double end_latency =
      (end_timestamp - in_message->measurement_time()) * 1e3;
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(radar_info_.name,
                                           "radar_perception");
  LOG_INFO << "FRAME_STATISTICS:Radar:End:msg_time["
           << GLOG_TIMESTAMP(in_message->measurement_time()) << "]:cur_time["
           << GLOG_TIMESTAMP(end_timestamp) << "]:cur_latency[" << end_latency
           << "]";

  return SUCC;
}

int RadarDetectionComponent::GetCarLocalizationSpeed(
    double timestamp, Eigen::Vector3f* car_linear_speed,
    Eigen::Vector3f* car_angular_speed) {
  CHECK_NOTNULL(car_linear_speed);
  (*car_linear_speed) = Eigen::Vector3f::Zero();
  CHECK_NOTNULL(car_angular_speed);
  (*car_angular_speed) = Eigen::Vector3f::Zero();
  std::shared_ptr<LocalizationEstimate const> loct_ptr;
  if (localization_subscriber_.LookupNearest(timestamp, &loct_ptr) != SUCC) {
    LOG_ERROR << "Can not get car speed.";
    return FAIL;
  }
  (*car_linear_speed)[0] = loct_ptr->pose().linear_velocity().x();
  (*car_linear_speed)[1] = loct_ptr->pose().linear_velocity().y();
  (*car_linear_speed)[2] = loct_ptr->pose().linear_velocity().z();
  (*car_angular_speed)[0] = loct_ptr->pose().angular_velocity().x();
  (*car_angular_speed)[1] = loct_ptr->pose().angular_velocity().y();
  (*car_angular_speed)[2] = loct_ptr->pose().angular_velocity().z();

  return SUCC;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
