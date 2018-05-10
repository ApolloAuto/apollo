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

#include "modules/perception/obstacle/radar/modest/modest_radar_detector.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/radar/modest/conti_radar_util.h"
#include "modules/perception/obstacle/radar/modest/object_builder.h"
#include "modules/perception/obstacle/radar/modest/radar_util.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromFile;

bool ModestRadarDetector::Init() {
  GetProtoFromFile(FLAGS_modest_radar_detector_config, &config_);

  if (FLAGS_use_navigation_mode) {
    config_.set_use_had_map(false);
  }

  if (!FLAGS_use_navigation_mode && !config_.has_use_had_map()) {
    AERROR << "use_had_map not found.";
    return false;
  }

  RadarTrack::SetTrackedTimesThreshold(config_.delay_frames());
  object_builder_.SetDelayFrame(config_.delay_frames());
  object_builder_.SetUseFpFilter(config_.use_fp_filter());

  conti_params_.probexist_vehicle = config_.probexist_vehicle();
  conti_params_.probexist_pedestrian = config_.probexist_pedestrian();
  conti_params_.probexist_bicycle = config_.probexist_bicycle();
  conti_params_.probexist_unknown = config_.probexist_unknown();
  conti_params_.lo_vel_rms_vehicle = config_.lo_vel_rms_vehicle();
  conti_params_.la_vel_rms_vehicle = config_.la_vel_rms_vehicle();
  conti_params_.lo_dist_rms_vehicle = config_.lo_dist_rms_vehicle();
  conti_params_.la_dist_rms_vehicle = config_.la_dist_rms_vehicle();
  conti_params_.lo_vel_rms_pedestrian = config_.lo_vel_rms_pedestrian();
  conti_params_.la_vel_rms_pedestrian = config_.la_vel_rms_pedestrian();
  conti_params_.lo_dist_rms_pedestrian = config_.lo_dist_rms_pedestrian();
  conti_params_.la_dist_rms_pedestrian = config_.la_dist_rms_pedestrian();
  conti_params_.lo_vel_rms_bicycle = config_.lo_vel_rms_bicycle();
  conti_params_.la_vel_rms_bicycle = config_.la_vel_rms_bicycle();
  conti_params_.lo_dist_rms_bicycle = config_.lo_dist_rms_bicycle();
  conti_params_.la_dist_rms_bicycle = config_.la_dist_rms_bicycle();
  conti_params_.lo_vel_rms_unknown = config_.lo_vel_rms_unknown();
  conti_params_.la_vel_rms_unknown = config_.la_vel_rms_unknown();
  conti_params_.lo_dist_rms_unknown = config_.lo_dist_rms_unknown();
  conti_params_.la_dist_rms_unknown = config_.la_dist_rms_unknown();

  object_builder_.SetContiParams(conti_params_);
  radar_tracker_.reset(new RadarTrackManager());

  AINFO << "Initialize the modest radar  detector";
  return true;
}

bool ModestRadarDetector::Detect(
    const ContiRadar &raw_obstacles,
    const std::vector<PolygonDType> &map_polygons,
    const RadarDetectorOptions &options,
    std::vector<std::shared_ptr<Object>> *objects) {
  if (objects == nullptr) {
    AERROR << "Objects is nullptr";
    return false;
  }
  ADEBUG << "Modest radar detector.";
  Eigen::Matrix4d radar_pose;
  if (options.radar2world_pose == nullptr) {
    AERROR << "radar2world_pose is nullptr.";
    return false;
  } else {
    radar_pose = *(options.radar2world_pose);
  }
  Eigen::Vector2d main_velocity;
  if (FLAGS_use_navigation_mode) {
    main_velocity[0] = 0;
    main_velocity[1] = 0;
  } else {
    main_velocity[0] = options.car_linear_speed[0];
    main_velocity[1] = options.car_linear_speed[1];
  }
  // preparation

  SensorObjects radar_objects;
  object_builder_.Build(raw_obstacles, radar_pose, main_velocity,
                        &radar_objects);
  radar_objects.timestamp =
      static_cast<double>(raw_obstacles.header().timestamp_sec());
  radar_objects.sensor_type = SensorType::RADAR;

  // roi filter
  auto &filter_objects = radar_objects.objects;
  RoiFilter(map_polygons, &filter_objects);
  // treatment
  radar_tracker_->Process(radar_objects);
  ADEBUG << "After process, object size: " << radar_objects.objects.size();
  CollectRadarResult(objects);
  ADEBUG << "radar object size: " << objects->size();
  return true;
}

bool ModestRadarDetector::CollectRadarResult(
    std::vector<std::shared_ptr<Object>> *objects) {
  std::vector<RadarTrack> &obs_track = radar_tracker_->GetTracks();
  if (objects == nullptr) {
    AERROR << "objects is nullptr";
    return false;
  }
  for (size_t i = 0; i < obs_track.size(); ++i) {
    std::shared_ptr<Object> object_ptr = std::shared_ptr<Object>(new Object());
    const std::shared_ptr<Object> &object_radar_ptr =
        obs_track[i].GetObsRadar();
    if (config_.use_fp_filter() && object_radar_ptr->is_background) {
      continue;
    }
    object_ptr->clone(*object_radar_ptr);
    object_ptr->tracking_time = obs_track[i].GetTrackingTime();
    object_ptr->track_id = obs_track[i].GetObsId();
    object_ptr->latest_tracked_time = obs_track[i].GetTimestamp();
    objects->push_back(object_ptr);
  }
  return true;
}

void ModestRadarDetector::RoiFilter(
    const std::vector<PolygonDType> &map_polygons,
    std::vector<std::shared_ptr<Object>> *filter_objects) {
  ADEBUG << "Before using hdmap, object size:" << filter_objects->size();
  // use new hdmap
  if (config_.use_had_map()) {
    if (!map_polygons.empty()) {
      int obs_number = 0;
      for (size_t i = 0; i < filter_objects->size(); i++) {
        pcl_util::PointD obs_position;
        obs_position.x = filter_objects->at(i)->center(0);
        obs_position.y = filter_objects->at(i)->center(1);
        obs_position.z = filter_objects->at(i)->center(2);
        if (RadarUtil::IsXyPointInHdmap<pcl_util::PointD>(obs_position,
                                                          map_polygons)) {
          filter_objects->at(obs_number) = filter_objects->at(i);
          obs_number++;
        }
      }
      filter_objects->resize(obs_number);
      ADEBUG << "query hdmap sucessfully!";
    } else {
      ADEBUG << "query hdmap unsuccessfully!";
    }
  }
  ADEBUG << "After using hdmap, object size:" << filter_objects->size();
}

}  // namespace perception
}  // namespace apollo
