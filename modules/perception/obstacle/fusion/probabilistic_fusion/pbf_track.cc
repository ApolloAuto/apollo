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

#include <algorithm>

#include "boost/format.hpp"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/macro.h"
#include "modules/common/time/time_util.h"
#include "modules/perception/common/geometry_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_track_object_matcher.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_imf_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

namespace apollo {
namespace perception {

/*class PbfTrack*/
int PbfTrack::s_track_idx_ = 0;
double PbfTrack::s_max_lidar_invisible_period_ = 0.25;
double PbfTrack::s_max_radar_invisible_period_ = 0.15;
double PbfTrack::s_max_camera_invisible_period_ = 0.25;
double PbfTrack::s_max_radar_confident_angle_ = 20;
double PbfTrack::s_min_radar_confident_distance_ = 40;

bool PbfTrack::s_publish_if_has_lidar_ = true;
bool PbfTrack::s_publish_if_has_radar_ = true;

using apollo::common::time::TimeUtil;

PbfTrack::MotionFusionMethod PbfTrack::s_motion_fusion_method_ =
    PbfTrack::MotionFusionMethod::PBF_KALMAN;

PbfTrack::PbfTrack(std::shared_ptr<PbfSensorObject> obj) {
  idx_ = GetNextTrackId();
  if (s_motion_fusion_method_ == MotionFusionMethod::PBF_KALMAN) {
    motion_fusion_.reset(new PbfKalmanMotionFusion());
  } else {
    motion_fusion_.reset(new PbfIMFFusion());
  }

  invisible_in_lidar_ = true;
  invisible_in_radar_ = true;
  invisible_in_camera_ = true;
  SensorType sensor_type = obj->sensor_type;
  std::string sensor_id = obj->sensor_id;
  if (is_lidar(sensor_type)) {
    lidar_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_lidar_ = false;
  } else if (is_radar(sensor_type)) {
    radar_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_radar_ = false;
  } else if (is_camera(sensor_type)) {
    camera_objects_[sensor_id] = obj;
    motion_fusion_->Initialize(obj);
    invisible_in_camera_ = false;
  } else {
    AERROR << "Unsupported sensor type : " << static_cast<int>(sensor_type)
           << ", sensor id : " << sensor_id;
  }

  motion_fusion_->setLastFuseTS(obj->timestamp);
  fused_timestamp_ = obj->timestamp;
  fused_object_.reset(new PbfSensorObject());
  fused_object_->clone(*obj);
  age_ = 0;
  invisible_period_ = 0;
  tracking_period_ = 0.0;
  is_dead_ = false;
}

void PbfTrack::SetMotionFusionMethod(const std::string &motion_fusion_method) {
  if (motion_fusion_method == "PbfKalmanMotionFusion") {
    s_motion_fusion_method_ = MotionFusionMethod::PBF_KALMAN;
  } else if (motion_fusion_method == "PbfIMFFusion") {
    s_motion_fusion_method_ = MotionFusionMethod::PBF_IMF;
  } else {
    AERROR << "Unknown motion fusion method.";
  }
}

PbfTrack::~PbfTrack() {}

void PbfTrack::UpdateWithSensorObject(std::shared_ptr<PbfSensorObject> obj,
                                      double match_dist) {
  const SensorType &sensor_type = obj->sensor_type;
  const std::string sensor_id = obj->sensor_id;
  if (FLAGS_async_fusion) {
    PerformMotionFusionAsync(obj);
  } else {
    PerformMotionFusion(obj);
  }

  if (is_lidar(sensor_type)) {
    lidar_objects_[sensor_id] = obj;
    invisible_in_lidar_ = false;
  } else if (is_radar(sensor_type)) {
    radar_objects_[sensor_id] = obj;
    invisible_in_radar_ = false;
  } else if (is_camera(sensor_type)) {
    camera_objects_[sensor_id] = obj;
    invisible_in_camera_ = false;
  }

  double timestamp = obj->timestamp;
  UpdateMeasurementsLifeWithMeasurement(&lidar_objects_, sensor_id, timestamp,
                                        s_max_lidar_invisible_period_);
  UpdateMeasurementsLifeWithMeasurement(&radar_objects_, sensor_id, timestamp,
                                        s_max_radar_invisible_period_);
  UpdateMeasurementsLifeWithMeasurement(&camera_objects_, sensor_id, timestamp,
                                        s_max_camera_invisible_period_);

  invisible_period_ = 0;
  tracking_period_ += obj->timestamp - fused_timestamp_;
  fused_timestamp_ = obj->timestamp;
  fused_object_->timestamp = obj->timestamp;
}

void PbfTrack::UpdateWithoutSensorObject(const SensorType &sensor_type,
                                         const std::string &sensor_id,
                                         double min_match_dist,
                                         double timestamp) {
  if (motion_fusion_ == nullptr) {
    AERROR << "Skip update becuase motion_fusion_ is nullptr.";
    return;
  }
  UpdateMeasurementsLifeWithoutMeasurement(
      &lidar_objects_, sensor_id, timestamp, s_max_lidar_invisible_period_,
      &invisible_in_lidar_);
  UpdateMeasurementsLifeWithoutMeasurement(
      &radar_objects_, sensor_id, timestamp, s_max_radar_invisible_period_,
      &invisible_in_radar_);
  UpdateMeasurementsLifeWithoutMeasurement(
      &camera_objects_, sensor_id, timestamp, s_max_camera_invisible_period_,
      &invisible_in_camera_);

  is_dead_ = (lidar_objects_.empty() && radar_objects_.empty() &&
              camera_objects_.empty());

  if (!is_dead_) {
    double time_diff = timestamp - fused_timestamp_;
    motion_fusion_->UpdateWithoutObject(time_diff);
    if (FLAGS_use_navigation_mode) {
      PerformMotionCompensation(fused_object_, timestamp);
    }
    invisible_period_ = timestamp - fused_timestamp_;
  }
}

int PbfTrack::GetTrackId() const { return idx_; }

std::shared_ptr<PbfSensorObject> PbfTrack::GetFusedObject() {
  return fused_object_;
}

double PbfTrack::GetFusedTimestamp() const { return fused_timestamp_; }

std::shared_ptr<PbfSensorObject> PbfTrack::GetLidarObject(
    const std::string &sensor_id) {
  std::shared_ptr<PbfSensorObject> obj = nullptr;
  auto it = lidar_objects_.find(sensor_id);
  if (it != lidar_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetRadarObject(
    const std::string &sensor_id) {
  std::shared_ptr<PbfSensorObject> obj = nullptr;
  auto it = radar_objects_.find(sensor_id);
  if (it != radar_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetCameraObject(
    const std::string &sensor_id) {
  std::shared_ptr<PbfSensorObject> obj = nullptr;
  auto it = camera_objects_.find(sensor_id);
  if (it != camera_objects_.end()) {
    obj = it->second;
  }
  return obj;
}

void PbfTrack::PerformMotionFusionAsync(std::shared_ptr<PbfSensorObject> obj) {
  if (motion_fusion_ == nullptr) {
    AERROR << "Skip motion fusion becuase motion_fusion_ is nullptr.";
    return;
  }
  AINFO << "perform motion fusion asynchrounously!";
  const SensorType &sensor_type = obj->sensor_type;

  double current_time = TimeUtil::GetCurrentTime();
  if (FLAGS_bag_mode) {
    // if running in bag, we can't estimate fusion arrival time correctly
    current_time =
        std::max(motion_fusion_->getLastFuseTS(), obj->timestamp) + 0.1;
    AINFO << "last fuse ts " << std::fixed << std::setprecision(15)
          << motion_fusion_->getLastFuseTS();
    AINFO << "obj timestamp " << std::fixed << std::setprecision(15)
          << obj->timestamp;
    AINFO << "current fuse ts is " << std::fixed << std::setprecision(15)
          << current_time;
  }

  // for low cost, we only consider radar and camera fusion for now
  if (is_camera(sensor_type) || is_radar(sensor_type)) {
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    motion_fusion_->setCurrentFuseTS(current_time);
    if (motion_fusion_->Initialized()) {
      double time_diff = motion_fusion_->getFuseTimeDiff();
      motion_fusion_->UpdateWithObject(obj, time_diff);
    } else {
      motion_fusion_->Initialize(obj);
      current_time = obj->timestamp;  // for initialize, we set fusion time to
                                      // sensor timestamp
    }
    Eigen::Vector3d anchor_point;
    motion_fusion_->GetState(&anchor_point, &velocity);
    fused_object_->object->velocity = velocity;
    fused_object_->object->anchor_point = anchor_point;
    fused_object_->object->center = anchor_point;
    // updated by arrival time of sensor object
    fused_object_->timestamp = current_time;
    ADEBUG << "fused object in pbftrack is "
           << fused_object_->object->ToString();
    motion_fusion_->setLastFuseTS(current_time);
  }
}

void PbfTrack::PerformMotionFusion(std::shared_ptr<PbfSensorObject> obj) {
  if (motion_fusion_ == nullptr) {
    AERROR << "Skip motion fusion becuase motion_fusion_ is nullptr.";
    return;
  }
  const SensorType &sensor_type = obj->sensor_type;
  double time_diff = obj->timestamp - fused_object_->timestamp;

  std::shared_ptr<PbfSensorObject> lidar_object = GetLatestLidarObject();
  std::shared_ptr<PbfSensorObject> radar_object = GetLatestRadarObject();

  if (FLAGS_use_navigation_mode) {
    if (is_camera(sensor_type) || is_radar(sensor_type)) {
      if (motion_fusion_->Initialized()) {
        motion_fusion_->UpdateWithObject(obj, time_diff);
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        // use radar position and velocity
        if (is_radar(sensor_type)) {
          motion_fusion_->SetState(obj->object->center, obj->object->velocity);
        }
        motion_fusion_->GetState(&anchor_point, &velocity);
        fused_object_->object->velocity = velocity;
        fused_object_->object->anchor_point = anchor_point;
        fused_object_->object->center = anchor_point;

        if (is_camera(sensor_type)) {
          fused_object_->object->theta = obj->object->theta;
          fused_object_->object->direction = obj->object->direction;
        }
      } else {
        if (is_camera(sensor_type)) {
          motion_fusion_->Initialize(obj);
        }
      }
    }
  } else {
    if (is_lidar(sensor_type)) {
      fused_object_->clone(*obj);
      if (motion_fusion_->Initialized() &&
          (lidar_object != nullptr || radar_object != nullptr)) {
        motion_fusion_->UpdateWithObject(obj, time_diff);
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        motion_fusion_->GetState(&anchor_point, &velocity);
        fused_object_->object->velocity = velocity;
      } else {
        motion_fusion_->Initialize(obj);
      }
    } else if (is_radar(sensor_type)) {
      if (motion_fusion_->Initialized() &&
          (lidar_object != nullptr || radar_object != nullptr)) {
        Eigen::Vector3d pre_anchor_point;
        Eigen::Vector3d pre_velocity;
        motion_fusion_->GetState(&pre_anchor_point, &pre_velocity);
        motion_fusion_->UpdateWithObject(obj, time_diff);
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        motion_fusion_->GetState(&anchor_point, &velocity);
        if (lidar_object == nullptr) {
          PbfSensorObject fused_obj_bk;
          fused_obj_bk.clone(*fused_object_);
          fused_object_->clone(*obj);
          fused_object_->object->velocity = velocity;
        } else {
          // if has lidar, use lidar shape
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
      AERROR << "Unsupport sensor type " << static_cast<int>(sensor_type);
      return;
    }
  }
}

void PbfTrack::PerformMotionCompensation(std::shared_ptr<PbfSensorObject> obj,
                                         double timestamp) {
  double time_diff = timestamp - obj->timestamp;
  if (time_diff < 0) {
    AERROR << "target timestamp is smaller than previous timestamp";
    return;
  }

  Eigen::Vector3d offset = obj->object->velocity * time_diff;
  obj->object->center += offset;
  obj->object->anchor_point += offset;

  PolygonDType &polygon = obj->object->polygon;
  for (size_t i = 0; i < polygon.size(); i++) {
    polygon.points[i].x += offset[0];
    polygon.points[i].y += offset[1];
    polygon.points[i].z += offset[2];
  }

  pcl_util::PointCloudPtr cloud = obj->object->cloud;
  for (size_t i = 0; i < cloud->size(); i++) {
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
    ++s_track_idx_;
  }
  return ret_track_id;
}

bool PbfTrack::AbleToPublish() {
  if (FLAGS_use_navigation_mode && !camera_objects_.empty()) {
    return true;
  }

  ADEBUG << s_publish_if_has_lidar_ << " " << invisible_in_lidar_ << " "
         << lidar_objects_.size();
  double invisible_period_threshold = 0.001;
  if (invisible_period_ > invisible_period_threshold && invisible_in_lidar_ &&
      invisible_in_radar_) {
    ADEBUG << "unable_to_publish: invisible " << invisible_period_;
    return false;
  }

  if (s_publish_if_has_lidar_ && !invisible_in_lidar_ &&
      !lidar_objects_.empty()) {
    return true;
  }

  std::shared_ptr<PbfSensorObject> radar_object = GetLatestRadarObject();
  if (s_publish_if_has_radar_ && !invisible_in_radar_ &&
      radar_object != nullptr) {
    if (radar_object->sensor_type == SensorType::RADAR) {
      if (radar_object->object->radar_supplement->range >
              s_min_radar_confident_distance_ &&
          radar_object->object->radar_supplement->angle <
              s_max_radar_confident_angle_) {
        if (fused_object_->object->velocity.dot(
                fused_object_->object->direction) < 0.3) {
          fused_object_->object->velocity.setZero();
          return false;
        }
        return true;
      }
    }
  }
  return false;
}

void PbfTrack::UpdateMeasurementsLifeWithMeasurement(
    std::map<std::string, std::shared_ptr<PbfSensorObject>> *objects,
    const std::string &sensor_id, double timestamp, double max_invisible_time) {
  for (auto it = objects->begin(); it != objects->end();) {
    if (sensor_id != it->first) {
      double period = timestamp - it->second->timestamp;
      if (period > max_invisible_time) {
        it->second = nullptr;
        it = objects->erase(it);
      } else {
        ++it;
      }
    } else {
      ++it;
    }
  }
}

void PbfTrack::UpdateMeasurementsLifeWithoutMeasurement(
    std::map<std::string, std::shared_ptr<PbfSensorObject>> *objects,
    const std::string &sensor_id, double timestamp, double max_invisible_time,
    bool *invisible_state) {
  *invisible_state = true;
  for (auto it = objects->begin(); it != objects->end();) {
    if (sensor_id == it->first) {
      it->second = nullptr;
      it = objects->erase(it);
    } else {
      double period = timestamp - it->second->timestamp;
      if (period > max_invisible_time) {
        it->second = nullptr;
        it = objects->erase(it);
      } else {
        *invisible_state = false;
        ++it;
      }
    }
  }
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetLatestLidarObject() {
  std::shared_ptr<PbfSensorObject> lidar_object;
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    if (lidar_object == nullptr) {
      lidar_object = it->second;
    } else if (lidar_object->timestamp < it->second->timestamp) {
      lidar_object = it->second;
    }
  }
  return lidar_object;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetLatestRadarObject() {
  std::shared_ptr<PbfSensorObject> radar_object;
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    if (radar_object == nullptr) {
      radar_object = it->second;
    } else if (radar_object->timestamp < it->second->timestamp) {
      radar_object = it->second;
    }
  }
  return radar_object;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetLatestCameraObject() {
  std::shared_ptr<PbfSensorObject> camera_object;
  for (auto it = camera_objects_.begin(); it != camera_objects_.end(); ++it) {
    if (camera_object == nullptr) {
      camera_object = it->second;
    } else if (camera_object->timestamp < it->second->timestamp) {
      camera_object = it->second;
    }
  }
  return camera_object;
}

std::shared_ptr<PbfSensorObject> PbfTrack::GetSensorObject(
    const SensorType &sensor_type, const std::string &sensor_id) {
  if (is_lidar(sensor_type)) {
    return GetLidarObject(sensor_id);
  } else if (is_radar(sensor_type)) {
    return GetRadarObject(sensor_id);
  } else if (is_camera(sensor_type)) {
    return GetCameraObject(sensor_id);
  } else {
    AERROR << "Unsupported sensor type.";
    return nullptr;
  }
}

}  // namespace perception
}  // namespace apollo
