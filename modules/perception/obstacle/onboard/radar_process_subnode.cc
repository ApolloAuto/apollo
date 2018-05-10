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

#include "modules/perception/obstacle/onboard/radar_process_subnode.h"

#include <algorithm>
#include <string>
#include <unordered_map>
#include "Eigen/Core"
#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/radar/dummy/dummy_algorithms.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/onboard/transform_input.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using Eigen::Affine3d;
using Eigen::Matrix4d;
using pcl_util::Point;
using pcl_util::PointD;
using std::string;
using std::unordered_map;

bool RadarProcessSubnode::InitInternal() {
  if (inited_) {
    return true;
  }

  RegistAllAlgorithm();

  if (!InitFrameDependence()) {
    AERROR << "failed to Init frame dependence.";
    return false;
  }

  if (!InitAlgorithmPlugin()) {
    AERROR << "failed to Init algorithm plugin.";
    return false;
  }
  // parse reserve fileds
  unordered_map<string, string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve filed: " << reserve_;
    return false;
  }

  if (reserve_field_map.find("device_id") == reserve_field_map.end()) {
    AERROR << "Failed to find field device_id, reserve: " << reserve_;
    return false;
  }
  device_id_ = reserve_field_map["device_id"];

  CHECK(AdapterManager::GetContiRadar()) << "Radar is not initialized.";
  AdapterManager::AddContiRadarCallback(&RadarProcessSubnode::OnRadar, this);
  CHECK(AdapterManager::GetLocalization()) << "Localiztion is not initialized.";
  AdapterManager::AddLocalizationCallback(&RadarProcessSubnode::OnLocalization,
                                          this);
  localization_buffer_.set_capacity(FLAGS_localization_buffer_size);
  std::string radar_extrinstic_path = FLAGS_radar_extrinsic_file;
  AINFO << "radar extrinsic path: " << radar_extrinstic_path;
  Eigen::Affine3d radar_extrinsic;
  if (!LoadExtrinsic(radar_extrinstic_path, &radar_extrinsic)) {
    AERROR << "Failed to load extrinsic.";
    return false;
  }
  radar_extrinsic_ = radar_extrinsic.matrix();
  AINFO << "get radar extrinsic succ. pose: \n" << radar_extrinsic_;

  std::string short_camera_extrinsic_path = FLAGS_short_camera_extrinsic_file;
  AINFO << "short camera extrinsic path: " << short_camera_extrinsic_path;
  Eigen::Affine3d short_camera_extrinsic;
  if (!LoadExtrinsic(short_camera_extrinsic_path, &short_camera_extrinsic)) {
    AERROR << "Failed to load extrinsic.";
    return false;
  }
  short_camera_extrinsic_ = short_camera_extrinsic.matrix();
  AINFO << "get short camera  extrinsic succ. pose: \n"
        << short_camera_extrinsic_;
  inited_ = true;

  return true;
}

void RadarProcessSubnode::OnRadar(const ContiRadar &radar_obs) {
  PERF_FUNCTION("RadarProcess");
  ContiRadar radar_obs_proto = radar_obs;
  double timestamp = radar_obs_proto.header().timestamp_sec();
  double unix_timestamp = timestamp;
  const double cur_time = common::time::Clock::NowInSeconds();
  const double start_latency = (cur_time - unix_timestamp) * 1e3;
  ADEBUG << "FRAME_STATISTICS: Radar:Start:msg_time["
         << GLOG_TIMESTAMP(timestamp) << "]:cur_time["
         << GLOG_TIMESTAMP(cur_time) << "]:cur_latency[" << start_latency
         << "]";
  // 0. correct radar timestamp
  timestamp -= 0.07;
  auto *header = radar_obs_proto.mutable_header();
  header->set_timestamp_sec(timestamp);
  header->set_radar_timestamp(timestamp * 1e9);

  conti_id_expansion_.UpdateTimestamp(timestamp);
  conti_id_expansion_.ExpandIds(&radar_obs_proto);

  if (fabs(timestamp - 0.0) < 10e-6) {
    AERROR << "Error timestamp: " << GLOG_TIMESTAMP(timestamp);
    return;
  }
  ADEBUG << "recv radar msg: [timestamp: " << GLOG_TIMESTAMP(timestamp)
         << " num_raw_obstacles: " << radar_obs_proto.contiobs_size() << "]";

  // 1. get radar pose
  std::shared_ptr<Matrix4d> velodyne2world_pose = std::make_shared<Matrix4d>();

  ADEBUG << "use navigation mode " << FLAGS_use_navigation_mode;

  if (!FLAGS_use_navigation_mode &&
      !GetVelodyneTrans(timestamp, velodyne2world_pose.get())) {
    AERROR << "Failed to get trans at timestamp: " << GLOG_TIMESTAMP(timestamp);
    error_code_ = common::PERCEPTION_ERROR_TF;
    return;
  }
  std::shared_ptr<Matrix4d> radar2world_pose = std::make_shared<Matrix4d>();
  std::shared_ptr<Matrix4d> radar2car_pose = std::make_shared<Matrix4d>();

  if (!FLAGS_use_navigation_mode) {
    *radar2world_pose =
        *velodyne2world_pose * short_camera_extrinsic_ * radar_extrinsic_;
    ADEBUG << "get radar trans pose succ. pose: \n" << *radar2world_pose;

  } else {
    CalibrationConfigManager *config_manager =
        Singleton<CalibrationConfigManager>::get();
    CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
    // Eigen::Matrix4d camera_to_car = calibrator->get_camera_extrinsics();
    *radar2car_pose = radar_extrinsic_;
    ADEBUG << "get radar trans pose succ. pose: \n" << *radar2car_pose;
  }

  std::vector<PolygonDType> map_polygons;
  RadarDetectorOptions options;
  // Current Localiztion, radar postion.
  if (!FLAGS_use_navigation_mode) {
    PointD position;
    position.x = (*radar2world_pose)(0, 3);
    position.y = (*radar2world_pose)(1, 3);
    position.z = (*radar2world_pose)(2, 3);
    // 2. Get map polygons.
    HdmapStructPtr hdmap(new HdmapStruct);
    if (FLAGS_enable_hdmap_input && hdmap_input_ &&
        !hdmap_input_->GetROI(position, FLAGS_front_radar_forward_distance,
                              &hdmap)) {
      AWARN << "Failed to get roi. timestamp: " << GLOG_TIMESTAMP(timestamp)
            << " position: [" << position.x << ", " << position.y << ", "
            << position.z << "]";
      // NOTE: if call hdmap failed, using empty map_polygons.
    }

    if (roi_filter_ != nullptr) {
      roi_filter_->MergeHdmapStructToPolygons(hdmap, &map_polygons);
    }

    // 3. get car car_linear_speed
    if (!GetCarLinearSpeed(timestamp, &(options.car_linear_speed))) {
      AERROR << "Failed to call get_car_linear_speed. [timestamp: "
             << GLOG_TIMESTAMP(timestamp);
      return;
    }
  }
  // 4. Call RadarDetector::detect.
  PERF_BLOCK_START();
  if (!FLAGS_use_navigation_mode) {
    options.radar2world_pose = &(*radar2world_pose);
  } else {
    options.radar2world_pose = &(*radar2car_pose);
  }
  std::shared_ptr<SensorObjects> radar_objects(new SensorObjects);
  radar_objects->timestamp = timestamp;
  radar_objects->sensor_type = SensorType::RADAR;
  radar_objects->sensor2world_pose = *radar2world_pose;
  bool result = radar_detector_->Detect(radar_obs_proto, map_polygons, options,
                                        &radar_objects->objects);
  if (!result) {
    radar_objects->error_code = common::PERCEPTION_ERROR_PROCESS;
    PublishDataAndEvent(timestamp, radar_objects);
    AERROR << "Failed to call RadarDetector. [timestamp: "
           << GLOG_TIMESTAMP(timestamp)
           << ", map_polygons_size: " << map_polygons.size()
           << ", num_raw_conti_obstacles: " << radar_obs_proto.contiobs_size()
           << "]";
    return;
  }
  PERF_BLOCK_END("radar_detect");
  PublishDataAndEvent(timestamp, radar_objects);

  const double end_timestamp = common::time::Clock::NowInSeconds();
  const double end_latency = (end_timestamp - unix_timestamp) * 1e3;
  ADEBUG << "FRAME_STATISTICS:Radar: End:msg_time[" << GLOG_TIMESTAMP(timestamp)
         << "]:cur_time[" << GLOG_TIMESTAMP(end_timestamp) << "]:cur_latency["
         << end_latency << "]";
  ADEBUG << "radar process succ, there are " << (radar_objects->objects).size()
         << " objects.";
  return;
}

void RadarProcessSubnode::OnLocalization(
    const apollo::localization::LocalizationEstimate &localization) {
  double timestamp = localization.header().timestamp_sec();
  ADEBUG << "localization timestamp:" << GLOG_TIMESTAMP(timestamp);
  LocalizationPair localization_pair;
  localization_pair.first = timestamp;
  localization_pair.second = localization;
  localization_buffer_.push_back(localization_pair);
}

bool RadarProcessSubnode::GetCarLinearSpeed(double timestamp,
                                            Eigen::Vector3f *car_linear_speed) {
  MutexLock lock(&mutex_);
  if (car_linear_speed == nullptr) {
    AERROR << "Param car_linear_speed nullptr error.";
    return false;
  }
  if (localization_buffer_.empty()) {
    AWARN << "Rosmsg buffer is empty.";
    return false;
  }
  if (localization_buffer_.front().first - 0.1 > timestamp) {
    AWARN << "Timestamp (" << GLOG_TIMESTAMP(timestamp)
          << ") is earlier than the oldest "
          << "timestamp (" << localization_buffer_.front().first << ").";
    return false;
  }
  if (localization_buffer_.back().first + 0.1 < timestamp) {
    AWARN << "Timestamp (" << GLOG_TIMESTAMP(timestamp)
          << ") is newer than the latest "
          << "timestamp (" << localization_buffer_.back().first << ").";
    return false;
  }
  // loop to find nearest
  double distance = 1e9;
  int idx = localization_buffer_.size() - 1;
  for (; idx >= 0; --idx) {
    double temp_distance = fabs(timestamp - localization_buffer_[idx].first);
    if (temp_distance >= distance) {
      break;
    }
    distance = temp_distance;
  }

  idx = std::max(0, idx);
  auto velocity = localization_buffer_[idx].second.pose().linear_velocity();
  (*car_linear_speed)[0] = velocity.x();
  (*car_linear_speed)[1] = velocity.y();
  (*car_linear_speed)[2] = velocity.z();
  return true;
}

void RadarProcessSubnode::RegistAllAlgorithm() {
  RegisterFactoryDummyRadarDetector();
  RegisterFactoryModestRadarDetector();
}

bool RadarProcessSubnode::InitFrameDependence() {
  /// init share data
  CHECK(shared_data_manager_ != nullptr) << "shared_data_manager_ is nullptr";
  // init preprocess_data
  const string radar_data_name("RadarObjectData");
  radar_data_ = dynamic_cast<RadarObjectData *>(
      shared_data_manager_->GetSharedData(radar_data_name));
  if (radar_data_ == nullptr) {
    AERROR << "Failed to get shared data instance " << radar_data_name;
    return false;
  }
  AINFO << "Init shared data successfully, data: " << radar_data_->name();

  /// init hdmap
  if (FLAGS_enable_hdmap_input) {
    hdmap_input_ = HDMapInput::instance();
    if (!hdmap_input_) {
      AERROR << "Failed to get HDMapInput instance.";
      return false;
    }
    if (!hdmap_input_->Init()) {
      AERROR << "Failed to Init HDMapInput";
      return false;
    }
    AINFO << "Get and Init hdmap_input succ.";
  }

  return true;
}

bool RadarProcessSubnode::InitAlgorithmPlugin() {
  /// init roi filter
  roi_filter_.reset(new HdmapROIFilter());
  if (!roi_filter_->Init()) {
    AERROR << "Failed to Init roi filter: " << roi_filter_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, roi_filter_: "
        << roi_filter_->name();

  /// init radar detector
  radar_detector_.reset(BaseRadarDetectorRegisterer::GetInstanceByName(
      FLAGS_onboard_radar_detector));
  if (!radar_detector_) {
    AERROR << "Failed to get instance: " << FLAGS_onboard_radar_detector;
    return false;
  }
  if (!radar_detector_->Init()) {
    AERROR << "Failed to Init radar detector: " << radar_detector_->name();
    return false;
  }
  AINFO << "Init algorithm plugin successfully, radar detecor: "
        << radar_detector_->name();
  return true;
}

void RadarProcessSubnode::PublishDataAndEvent(
    double timestamp, const SharedDataPtr<SensorObjects> &data) {
  // set shared data
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "Failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return;
  }

  radar_data_->Add(key, data);
  // pub events
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

}  // namespace perception
}  // namespace apollo
