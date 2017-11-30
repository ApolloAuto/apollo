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

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/radar/modest/conti_radar_util.h"
#include "modules/perception/obstacle/radar/modest/radar_util.h"
#include "modules/perception/obstacle/radar/modest/object_builder.h"

namespace apollo {
namespace perception {

bool ModestRadarDetector::Init() {
  using apollo::perception::ConfigManager;
  using apollo::perception::ModelConfig;
  const ModelConfig *model_config = nullptr;
  if (!ConfigManager::instance()->GetModelConfig(name(), &model_config)) {
    AERROR << "not found model config: " << name();
    return false;
  }
  if (!model_config->GetValue("use_had_map", &use_had_map_)) {
    AERROR << "use_had_map not found.";
    return false;
  }
  if (!model_config->GetValue("max_theta", &max_theta_)) {
    AERROR << "max_theta not found.";
    return false;
  }
  if (!model_config->GetValue("delay_frames", &delay_frames_)) {
    AERROR << "delay_frame not found.";
    return false;
  }
  RadarTrack::SetTrackedTimesThreshold(delay_frames_);
  object_builder_.SetDelayFrame(delay_frames_);
  if (!model_config->GetValue("use_fp_filter", &use_fp_filter_)) {
    AERROR << "use_fp_filter is not found.";
    return false;
  }
  object_builder_.SetUseFpFilter(use_fp_filter_);
  if (!model_config->GetValue("probexist_vehicle",
                              &(conti_params_.probexist_vehicle))) {
    AERROR << "probexist_vehicle not found.";
    return false;
  }
  if (!model_config->GetValue("probexist_pedestrian",
                              &(conti_params_.probexist_pedestrian))) {
    AERROR << "probexist_pedestrian not found.";
    return false;
  }
  if (!model_config->GetValue("probexist_bicycle",
                              &(conti_params_.probexist_bicycle))) {
    AERROR << "probexist_bicycle not found.";
    return false;
  }
  if (!model_config->GetValue("probexist_unknown",
                              &(conti_params_.probexist_unknown))) {
    AERROR << "probexist_unknown not found.";
    return false;
  }
  if (!model_config->GetValue("lo_vel_rms_vehicle",
                              &(conti_params_.lo_vel_rms_vehicle))) {
    AERROR << "lo_vel_rms_vehicle not found.";
    return false;
  }
  if (!model_config->GetValue("la_vel_rms_vehicle",
                              &(conti_params_.la_vel_rms_vehicle))) {
    AERROR << "la_vel_rms_vehicle not found.";
    return false;
  }
  if (!model_config->GetValue("lo_dist_rms_vehicle",
                              &(conti_params_.lo_dist_rms_vehicle))) {
    AERROR << "lo_dist_rms_vehicle not found.";
    return false;
  }
  if (!model_config->GetValue("la_dist_rms_vehicle",
                              &(conti_params_.la_dist_rms_vehicle))) {
    AERROR << "la_vel_dist_vehicle not found.";
    return false;
  }
  if (!model_config->GetValue("lo_vel_rms_pedestrian",
                              &(conti_params_.lo_vel_rms_pedestrian))) {
    AERROR << "lo_vel_rms_pedestrian not found.";
    return false;
  }
  if (!model_config->GetValue("la_vel_rms_pedestrian",
                              &(conti_params_.la_vel_rms_pedestrian))) {
    AERROR << "la_vel_rms_vehicle not found.";
    return false;
  }
  if (!model_config->GetValue("lo_dist_rms_pedestrian",
                              &(conti_params_.lo_dist_rms_pedestrian))) {
    AERROR << "lo_dist_rms_pedestrian not found.";
    return false;
  }
  if (!model_config->GetValue("la_dist_rms_pedestrian",
                              &(conti_params_.la_dist_rms_pedestrian))) {
    AERROR << "la_vel_dist_pedestrian not found.";
    return false;
  }
  if (!model_config->GetValue("lo_vel_rms_bicycle",
                              &(conti_params_.lo_vel_rms_bicycle))) {
    AERROR << "lo_vel_rms_bicycle not found.";
    return false;
  }
  if (!model_config->GetValue("la_vel_rms_bicycle",
                              &(conti_params_.la_vel_rms_bicycle))) {
    AERROR << "la_vel_rms_bicycle not found.";
    return false;
  }
  if (!model_config->GetValue("lo_dist_rms_bicycle",
                              &(conti_params_.lo_dist_rms_bicycle))) {
    AERROR << "lo_dist_rms_bicycle not found.";
    return false;
  }
  if (!model_config->GetValue("la_dist_rms_bicycle",
                              &(conti_params_.la_dist_rms_bicycle))) {
    AERROR << "la_vel_dist_bicycle not found.";
    return false;
  }
  if (!model_config->GetValue("lo_vel_rms_unknown",
                              &(conti_params_.lo_vel_rms_unknown))) {
    AERROR << "lo_vel_rms_unknown not found.";
    return false;
  }
  if (!model_config->GetValue("la_vel_rms_unknown",
                              &(conti_params_.la_vel_rms_unknown))) {
    AERROR << "la_vel_rms_unkown not found.";
    return false;
  }
  if (!model_config->GetValue("lo_dist_rms_unknown",
                              &(conti_params_.lo_dist_rms_unknown))) {
    AERROR << "lo_dist_rms_unknown not found.";
    return false;
  }
  if (!model_config->GetValue("la_dist_rms_unknown",
                              &(conti_params_.la_dist_rms_unknown))) {
    AERROR << "la_vel_dist_unknown not found.";
    return false;
  }
  object_builder_.SetContiParams(conti_params_);
  radar_tracker_.reset(new RadarTrackManager());
  AINFO << "Initialize the modest radar  detector";
  return true;
}

bool ModestRadarDetector::Detect(const ContiRadar &raw_obstacles,
                                 const std::vector<PolygonDType> &map_polygons,
                                 const RadarDetectorOptions &options,
                                 std::vector<ObjectPtr> *objects) {
  if (objects == nullptr) {
    AERROR << "Objects is nullptr";
    return false;
  }
  AINFO << "Modest radar detector.";
  Eigen::Matrix4d radar_pose;
  if (options.radar2world_pose == nullptr) {
    AERROR << "radar2world_pose is nullptr.";
    return false;
  } else {
    radar_pose = *(options.radar2world_pose);
  }
  Eigen::Vector2d main_velocity;
  main_velocity[0] = options.car_linear_speed[0];
  main_velocity[1] = options.car_linear_speed[1];
  // preparation

  std::shared_ptr<SensorObjects> radar_objects(new SensorObjects);
  object_builder_.Build(raw_obstacles, radar_pose, main_velocity, *radar_objects);
  radar_objects->timestamp = (double) raw_obstacles.header().timestamp_sec();
  radar_objects->sensor_type = RADAR;

  // roi filter
  auto &filter_objects = radar_objects->objects;
  RoiFilter(map_polygons, filter_objects);
  // treatment
  radar_tracker_->Process(*radar_objects);
  AINFO << "After process, object size: " << radar_objects->objects.size();
  CollectRadarResult(objects);
  AINFO << "radar object size: " << objects->size();
  return true;
}

bool ModestRadarDetector::CollectRadarResult(std::vector<ObjectPtr> *objects) {
  std::vector<RadarTrack> &obs_track = radar_tracker_->GetTracks();
  if (objects == nullptr) {
    AERROR << "objects is nullptr";
    return false;
  }
  for (size_t i = 0; i < obs_track.size(); ++i) {
    ObjectPtr object_ptr = ObjectPtr(new Object());
    const ObjectPtr &object_radar_ptr = obs_track[i].GetObsRadar();
    if (object_radar_ptr->is_background) {
      continue;
    }
    object_ptr->clone(*object_radar_ptr);
    object_ptr->tracking_time = obs_track[i].GetTrackingTime();
    object_ptr->track_id = obs_track[i].GetObsId();
    objects->push_back(object_ptr);
  }
  return true;
}

void ModestRadarDetector::RoiFilter(const std::vector<PolygonDType> &map_polygons,
                                    std::vector<ObjectPtr> &filter_objects) {
  AINFO << "Before using hdmap, object size:" << filter_objects.size();
  // use new hdmap
  if (use_had_map_) {
    if (!map_polygons.empty()) {
      int obs_number = 0;
      for (size_t i = 0; i < filter_objects.size(); i++) {
        pcl_util::PointD obs_position;
        obs_position.x = filter_objects[i]->center(0);
        obs_position.y = filter_objects[i]->center(1);
        obs_position.z = filter_objects[i]->center(2);
        if (RadarUtil::IsXyPointInHdmap<pcl_util::PointD>(
            obs_position, map_polygons)) {
          filter_objects[obs_number] = filter_objects[i];
          obs_number++;
        }
      }
      filter_objects.resize(obs_number);
      AINFO << "query hdmap sucessfully!";
    } else {
      AINFO << "query hdmap unsuccessfully!";
    }
  }
  AINFO << "After using hdmap, object size:" << filter_objects.size();
}

}  // namespace perception
}  // namespace apollo
