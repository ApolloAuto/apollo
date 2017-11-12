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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

#include "boost/format.hpp"

#include "modules/perception/obstacle/base/types.h"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"

namespace apollo {
namespace perception {
/*class PbfTrack*/
int PbfTrack::s_track_idx_ = 0;
double PbfTrack::s_max_lidar_invisible_period_ = 0.25;
double PbfTrack::s_max_radar_invisible_period_ = 0.15;
double PbfTrack::s_max_radar_confident_angle_ = 20;
double PbfTrack::s_min_radar_confident_distance_ = 40;
std::string PbfTrack::s_motion_fusion_method_ = "PbfKalmanMotionFusion";

bool PbfTrack::s_publish_if_has_lidar_ = true;
bool PbfTrack::s_publish_if_has_radar_ = true;

PbfTrack::PbfTrack(PbfSensorObjectPtr obj) {
  idx_ = GetNextTrackId();
  SensorType sensor_type = obj->sensor_type;
  std::string sensor_id = obj->sensor_id;
  invisible_in_lidar_ = true;
  invisible_in_radar_ = true;

  if (s_motion_fusion_method_ == "PbfKalmanMotionFusion") {
    motion_fusion_ = new PbfKalmanMotionFusion();
  } else {
    motion_fusion_ = new PbfKalmanMotionFusion();
  }

  if (is_lidar(sensor_type)) {
    lidar_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_lidar_ = false;
  } else if (is_radar(sensor_type)) {
    radar_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_radar_ = false;
  } else {
    AERROR << "Unsupported sensor type : "
           << sensor_type << ", sensor id : " << sensor_id;
  }

  fused_timestamp_ = obj->timestamp;
  fused_object_.reset(new PbfSensorObject());
  fused_object_->clone(*obj);
  age_ = 0;
  invisible_period_ = 0;
  tracking_period_ = 0.0;
  is_dead_ = false;
}

void PbfTrack::SetMotionFusionMethod(const std::string motion_fusion_method) {
  if (motion_fusion_method == "PbfKalmanMotionFusion") {
    s_motion_fusion_method_ = motion_fusion_method;
  } else {
    AERROR << "unsupported motion fusion method : " << motion_fusion_method
           << ", use default method : " << s_motion_fusion_method_;
  }
}

PbfTrack::~PbfTrack() {
  if (motion_fusion_ != nullptr) {
    delete motion_fusion_;
    motion_fusion_ = nullptr;
  }
}

void PbfTrack::UpdateWithSensorObject(PbfSensorObjectPtr obj, double match_dist) {

  const SensorType &sensor_type = obj->sensor_type;
  const std::string sensor_id = obj->sensor_id;
  PerformMotionFusion(obj);

  if (is_lidar(sensor_type)) {
    lidar_objects_[sensor_id] = obj;
    invisible_in_lidar_ = false;
  } else if (is_radar(sensor_type)) {
    radar_objects_[sensor_id] = obj;
    invisible_in_radar_ = false;
  }

  double timestamp = obj->timestamp;
  UpdateMeasurementsLifeWithMeasurement(lidar_objects_, sensor_id,
                                        timestamp, s_max_lidar_invisible_period_);
  UpdateMeasurementsLifeWithMeasurement(radar_objects_, sensor_id,
                                        timestamp, s_max_radar_invisible_period_);

  invisible_period_ = 0;
  tracking_period_ += obj->timestamp - fused_timestamp_;
  fused_timestamp_ = obj->timestamp;
  fused_object_->timestamp = obj->timestamp;
}
void PbfTrack::UpdateWithoutSensorObject(const SensorType &sensor_type,
                                         const std::string &sensor_id, double min_match_dist, double timestamp) {
  bool is_alive = false;
  UpdateMeasurementsLifeWithoutMeasurement(lidar_objects_,
                                           sensor_id, timestamp, s_max_lidar_invisible_period_, invisible_in_lidar_);
  UpdateMeasurementsLifeWithoutMeasurement(radar_objects_,
                                           sensor_id, timestamp, s_max_radar_invisible_period_, invisible_in_radar_);

  is_alive = (!lidar_objects_.empty() || !radar_objects_.empty());
  is_dead_ = !is_alive;

  if (is_alive) {
    double time_diff = timestamp - fused_timestamp_;
    motion_fusion_->UpdateWithoutObject(time_diff);
    PerformMotionCompensation(fused_object_, timestamp);
    invisible_period_ = timestamp - fused_timestamp_;
  }
}

int PbfTrack::GetTrackId() const {
  return idx_;
}

PbfSensorObjectPtr PbfTrack::GetFusedObject() {
  return fused_object_;
}

double PbfTrack::GetFusedTimestamp() const {
  return fused_timestamp_;
}

PbfSensorObjectPtr PbfTrack::GetLidarObject(const std::string &sensor_id) {
  PbfSensorObjectPtr obj = nullptr;
  auto it = lidar_objects_.find(sensor_id);
  if (it != lidar_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

PbfSensorObjectPtr PbfTrack::GetRadarObject(const std::string &sensor_id) {
  PbfSensorObjectPtr obj = nullptr;
  auto it = radar_objects_.find(sensor_id);
  if (it != radar_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

void PbfTrack::PerformMotionFusion(PbfSensorObjectPtr obj) {

  const SensorType &sensor_type = obj->sensor_type;
  double time_diff = obj->timestamp - fused_object_->timestamp;

  PbfSensorObjectPtr lidar_object = GetLatestLidarObject();
  PbfSensorObjectPtr radar_object = GetLatestRadarObject();

  if (is_lidar(sensor_type)) {
    fused_object_->clone(*obj);
    if (motion_fusion_->Initialized() &&
        (lidar_object != nullptr || radar_object != nullptr)) {
      motion_fusion_->UpdateWithObject(obj, time_diff);
      Eigen::Vector3d anchor_point;
      Eigen::Vector3d velocity;
      motion_fusion_->GetState(anchor_point, velocity);
      fused_object_->object->velocity = velocity;
    } else {
      motion_fusion_->Initialize(obj);
    }
  } else if (is_radar(sensor_type)) {
    if (motion_fusion_->Initialized() &&
        (lidar_object != nullptr || radar_object != nullptr)) {
      Eigen::Vector3d pre_anchor_point;
      Eigen::Vector3d pre_velocity;
      motion_fusion_->GetState(pre_anchor_point, pre_velocity);
      motion_fusion_->UpdateWithObject(obj, time_diff);
      Eigen::Vector3d anchor_point;
      Eigen::Vector3d velocity;
      motion_fusion_->GetState(anchor_point, velocity);
      if (lidar_object == nullptr) {
        PbfSensorObject fused_obj_bk;
        fused_obj_bk.clone(*fused_object_);
        fused_object_->clone(*obj);
        fused_object_->object->velocity = velocity;
      } else {
        //if has lidar, use lidar shape
        Eigen::Vector3d translation = anchor_point - pre_anchor_point;
        fused_object_->object->anchor_point = anchor_point;
        fused_object_->object->center += translation;
        for (auto point : fused_object_->object->polygon.points) {
          point.x += translation[0];
          point.y += translation[1];
          point.z += translation[2];
        }
        fused_object_->object->velocity = velocity;
      }
    } else {
      AERROR << "Something must be wrong.";
    }
  } else {
    AERROR << "Unsupport sensor type " << sensor_type;
    return;
  }

}

void PbfTrack::PerformMotionCompensation(PbfSensorObjectPtr obj, double timestamp) {
  double time_diff = timestamp - obj->timestamp;
  if (time_diff < 0) {
    AERROR << "target timestamp is smaller than previous timestamp";
    return;
  }

  Eigen::Vector3d offset = obj->object->velocity * time_diff;
  obj->object->center += offset;
  obj->object->anchor_point += offset;

  PolygonDType &polygon = obj->object->polygon;
  for (int i = 0; i < (int) polygon.size(); i++) {
    polygon.points[i].x += offset[0];
    polygon.points[i].y += offset[1];
    polygon.points[i].z += offset[2];
  }

  pcl_util::PointCloudPtr cloud = obj->object->cloud;
  for (int i = 0; i < (int) cloud->size(); i++) {
    cloud->points[i].x += offset[0];
    cloud->points[i].y += offset[1];
    cloud->points[i].z += offset[2];
  }

  obj->timestamp = timestamp;
  obj->object->latest_tracked_time = timestamp;
  obj->object->tracking_time += time_diff;
}

int PbfTrack::GetNextTrackId() {
  int ret_track_id = s_track_idx_;
  if (s_track_idx_ == INT_MAX) {
    s_track_idx_ = 0;
  } else {
    s_track_idx_++;
  }
  return ret_track_id;
}

bool PbfTrack::AbleToPublish() {
  AINFO << s_publish_if_has_lidar_ << " " << invisible_in_lidar_ << " " << lidar_objects_.size();
  double invisible_period_threshold = 0.001;
  if (invisible_period_ > invisible_period_threshold &&
      invisible_in_lidar_ && invisible_in_radar_) {
    ADEBUG << "unable_to_publish: invisible " << invisible_period_;
    return false;
  }

  if (s_publish_if_has_lidar_ && !invisible_in_lidar_ && !lidar_objects_.empty()) {
    return true;
  }

  PbfSensorObjectPtr radar_object = GetLatestRadarObject();
  if (s_publish_if_has_radar_ && !invisible_in_radar_ && radar_object != nullptr) {
    if (radar_object->sensor_type == RADAR) {
      if (radar_object->object->radar_supplement->range > s_min_radar_confident_distance_ &&
          radar_object->object->radar_supplement->angle < s_max_radar_confident_angle_) {
        if (fused_object_->object->velocity.dot(fused_object_->object->direction) < 0.3) {
          fused_object_->object->velocity.setZero();
        }
        return true;
      }
    }
  }
  return false;
}

void PbfTrack::UpdateMeasurementsLifeWithMeasurement(
    std::map<std::string, PbfSensorObjectPtr> &objects,
    const std::string &sensor_id, double timestamp, double max_invisible_time) {
  for (auto it = objects.begin(); it != objects.end();) {
    if (sensor_id != it->first) {
      double period = timestamp - it->second->timestamp;
      if (period > max_invisible_time) {
        it->second = nullptr;
        it = objects.erase(it);
      } else {
        ++it;
      }
    } else {
      ++it;
    }
  }
}

void PbfTrack::UpdateMeasurementsLifeWithoutMeasurement(
    std::map<std::string, PbfSensorObjectPtr> &objects,
    const std::string &sensor_id, double timestamp, double max_invisible_time,
    bool &invisible_state) {

  invisible_state = true;
  for (auto it = objects.begin(); it != objects.end();) {
    if (sensor_id == it->first) {
      it->second = nullptr; //TODO: consider in-view state
      it = objects.erase(it);
    } else {
      double period = timestamp - it->second->timestamp;
      if (period > max_invisible_time) {
        it->second = nullptr;
        it = objects.erase(it);
      } else {
        invisible_state = false;
        ++it;
      }
    }
  }
}

PbfSensorObjectPtr PbfTrack::GetLatestLidarObject() {
  PbfSensorObjectPtr lidar_object;
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    if (lidar_object == nullptr) {
      lidar_object = it->second;
    } else if (lidar_object->timestamp < it->second->timestamp) {
      lidar_object = it->second;
    }
  }
  return lidar_object;
}

PbfSensorObjectPtr PbfTrack::GetLatestRadarObject() {
  PbfSensorObjectPtr radar_object;
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    if (radar_object == nullptr) {
      radar_object = it->second;
    } else if (radar_object->timestamp < it->second->timestamp) {
      radar_object = it->second;
    }
  }
  return radar_object;
}

PbfSensorObjectPtr PbfTrack::GetSensorObject(const SensorType &sensor_type,
                                             const std::string &sensor_id) {
  if (is_lidar(sensor_type)) {
    return GetLidarObject(sensor_id);
  } else if (is_radar(sensor_type)) {
    return GetRadarObject(sensor_id);
  } else {
    AERROR << "Unsupported sensor type.";
    return nullptr;
  }
}

} // namespace perception
} // namespace apollo
