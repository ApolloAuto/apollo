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
#include "modules/perception/multi_sensor_fusion/base/track.h"

#include <limits>

namespace apollo {
namespace perception {
namespace fusion {

size_t Track::s_track_idx_ = 1;
double Track::s_max_lidar_invisible_period_ = 0.25;   // in second
double Track::s_max_radar_invisible_period_ = 0.50;   // in second
double Track::s_max_camera_invisible_period_ = 0.75;  // in second

Track::Track() { fused_object_.reset(new FusedObject()); }

bool Track::Initialize(SensorObjectPtr obj, bool is_background) {
  Reset();
  int track_id = static_cast<int>(GenerateNewTrackId());
  is_background_ = is_background;
  std::shared_ptr<base::Object> fused_base_obj = fused_object_->GetBaseObject();
  std::shared_ptr<const base::Object> sensor_base_obj = obj->GetBaseObject();
  *fused_base_obj = *sensor_base_obj;
  fused_base_obj->track_id = track_id;
  UpdateWithSensorObject(obj);
  return true;
}

void Track::Reset() {
  fused_object_->GetBaseObject()->track_id = 0;
  lidar_objects_.clear();
  radar_objects_.clear();
  camera_objects_.clear();
  tracking_period_ = 0.0;
  is_background_ = false;
  is_alive_ = true;
  tracked_times_ = 0;
  toic_prob_ = 0.0;
}

SensorObjectConstPtr Track::GetSensorObject(
    const std::string& sensor_id) const {
  auto lidar_it = lidar_objects_.find(sensor_id);
  if (lidar_it != lidar_objects_.end()) {
    return lidar_it->second;
  }

  auto radar_it = radar_objects_.find(sensor_id);
  if (radar_it != radar_objects_.end()) {
    return radar_it->second;
  }

  auto camera_it = camera_objects_.find(sensor_id);
  if (camera_it != camera_objects_.end()) {
    return camera_it->second;
  }

  return nullptr;
}

SensorObjectConstPtr Track::GetLatestLidarObject() const {
  return GetLatestSensorObject(lidar_objects_);
}

SensorObjectConstPtr Track::GetLatestRadarObject() const {
  return GetLatestSensorObject(radar_objects_);
}

SensorObjectConstPtr Track::GetLatestCameraObject() const {
  return GetLatestSensorObject(camera_objects_);
}

SensorObjectConstPtr Track::GetLatestSensorObject(
    const SensorId2ObjectMap& objects) const {
  SensorObjectConstPtr obj = nullptr;
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    if (obj == nullptr || obj->GetTimestamp() < it->second->GetTimestamp()) {
      obj = it->second;
    }
  }
  return obj;
}

size_t Track::GenerateNewTrackId() {
  int ret_track_id = static_cast<int>(s_track_idx_);
  if (s_track_idx_ == std::numeric_limits<unsigned int>::max()) {
    s_track_idx_ = 1;
  } else {
    s_track_idx_++;
  }
  return ret_track_id;
}

void Track::UpdateSensorObject(SensorId2ObjectMap* objects,
                               const SensorObjectPtr& obj) {
  std::string sensor_id = obj->GetSensorId();
  auto it = objects->find(sensor_id);
  if (it == objects->end()) {
    (*objects)[sensor_id] = obj;
  } else {
    it->second = obj;
  }
}

void Track::UpdateWithSensorObject(const SensorObjectPtr& obj) {
  std::string sensor_id = obj->GetSensorId();
  SensorId2ObjectMap* objects = nullptr;
  if (IsLidar(obj)) {
    objects = &lidar_objects_;
  } else if (IsRadar(obj)) {
    objects = &radar_objects_;
  } else if (IsCamera(obj)) {
    objects = &camera_objects_;
  } else {
    return;
  }
  UpdateSensorObject(objects, obj);
  double time_diff = obj->GetTimestamp() - fused_object_->GetTimestamp();
  tracking_period_ += time_diff;

  UpdateSensorObjectWithMeasurement(&lidar_objects_, sensor_id,
                                    obj->GetTimestamp(),
                                    s_max_lidar_invisible_period_);
  UpdateSensorObjectWithMeasurement(&radar_objects_, sensor_id,
                                    obj->GetTimestamp(),
                                    s_max_radar_invisible_period_);
  UpdateSensorObjectWithMeasurement(&camera_objects_, sensor_id,
                                    obj->GetTimestamp(),
                                    s_max_camera_invisible_period_);

  if (is_background_) {
    return UpdateWithSensorObjectForBackground(obj);
  }

  fused_object_->GetBaseObject()->latest_tracked_time = obj->GetTimestamp();
  UpdateSupplementState(obj);
  UpdateUnfusedState(obj);
  is_alive_ = true;
}

void Track::UpdateWithoutSensorObject(const std::string& sensor_id,
                                      double measurement_timestamp) {
  UpdateSensorObjectWithoutMeasurement(&lidar_objects_, sensor_id,
                                       measurement_timestamp,
                                       s_max_lidar_invisible_period_);
  UpdateSensorObjectWithoutMeasurement(&radar_objects_, sensor_id,
                                       measurement_timestamp,
                                       s_max_radar_invisible_period_);
  UpdateSensorObjectWithoutMeasurement(&camera_objects_, sensor_id,
                                       measurement_timestamp,
                                       s_max_camera_invisible_period_);

  UpdateSupplementState();
  is_alive_ = (!lidar_objects_.empty()) || (!radar_objects_.empty()) ||
              (!camera_objects_.empty());
}

void Track::UpdateSensorObjectWithoutMeasurement(SensorId2ObjectMap* objects,
                                                 const std::string& sensor_id,
                                                 double measurement_timestamp,
                                                 double max_invisible_period) {
  for (auto it = objects->begin(); it != objects->end();) {
    double period = measurement_timestamp - it->second->GetTimestamp();
    if (it->first == sensor_id) {
      it->second->SetInvisiblePeriod(period);
    } else if (it->second->GetInvisiblePeriod() > 0.0) {
      it->second->SetInvisiblePeriod(period);
    }

    if (period > max_invisible_period) {
      it->second = nullptr;
      it = objects->erase(it);
    } else {
      ++it;
    }
  }
}

void Track::UpdateSensorObjectWithMeasurement(SensorId2ObjectMap* objects,
                                              const std::string& sensor_id,
                                              double measurement_timestamp,
                                              double max_invisible_period) {
  for (auto it = objects->begin(); it != objects->end();) {
    if (it->first != sensor_id) {
      double period = measurement_timestamp - it->second->GetTimestamp();
      if (period > max_invisible_period) {
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

void Track::UpdateSupplementState(const SensorObjectPtr& src_object) {
  std::shared_ptr<base::Object> dst_obj = fused_object_->GetBaseObject();
  if (src_object != nullptr) {
    std::shared_ptr<const base::Object> src_obj = src_object->GetBaseObject();
    if (IsLidar(src_object)) {
      dst_obj->lidar_supplement = src_obj->lidar_supplement;
    } else if (IsRadar(src_object)) {
      dst_obj->radar_supplement = src_obj->radar_supplement;
    } else if (IsCamera(src_object)) {
      dst_obj->camera_supplement = src_obj->camera_supplement;
    }
  }

  if (lidar_objects_.empty()) {
    dst_obj->lidar_supplement.Reset();
  }
  if (radar_objects_.empty()) {
    dst_obj->radar_supplement.Reset();
  }
  if (camera_objects_.empty()) {
    dst_obj->camera_supplement.Reset();
  }
}

void Track::UpdateUnfusedState(const SensorObjectPtr& src_object) {
  std::shared_ptr<base::Object> dst_obj = fused_object_->GetBaseObject();
  std::shared_ptr<const base::Object> src_obj = src_object->GetBaseObject();
  if (IsLidar(src_object)) {
    dst_obj->confidence = src_obj->confidence;
    dst_obj->velocity_converged = src_obj->velocity_converged;
  } else if (IsRadar(src_object)) {
    // update nothing temporarily
  } else if (IsCamera(src_object)) {
    dst_obj->confidence = src_obj->confidence;
  }
}

bool Track::IsVisible(const std::string& sensor_id) const {
  SensorObjectConstPtr sensor_obj = GetSensorObject(sensor_id);
  return (sensor_obj != nullptr && sensor_obj->GetInvisiblePeriod() < 1.0e-6);
}

bool Track::IsLidarVisible() const {
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < 1.0e-6) {
      return true;
    }
  }
  return false;
}

bool Track::IsRadarVisible() const {
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < 1.0e-6) {
      return true;
    }
  }
  return false;
}

bool Track::IsCameraVisible() const {
  for (auto it = camera_objects_.begin(); it != camera_objects_.end(); ++it) {
    if (it->second->GetInvisiblePeriod() < 1.0e-6) {
      return true;
    }
  }
  return false;
}

void Track::UpdateWithSensorObjectForBackground(const SensorObjectPtr& obj) {
  std::shared_ptr<base::Object> fused_base_object =
      fused_object_->GetBaseObject();
  std::shared_ptr<const base::Object> measurement_base_object =
      obj->GetBaseObject();
  int track_id = fused_base_object->track_id;
  *fused_base_object = *measurement_base_object;
  fused_base_object->track_id = track_id;
}

void Track::UpdateWithoutSensorObjectForBackground(
    const std::string& sensor_id, double measurement_timestamp) {}

std::string Track::DebugString() const {
  std::ostringstream oss;
  oss << "fusion_track[id: " << this->GetTrackId() << ", fused_object("
      << fused_object_->GetBaseObject()->ToString() << ")\n";

  oss << "lidar_measurments[number: " << lidar_objects_.size() << ",";
  for (auto it = lidar_objects_.begin(); it != lidar_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first
        << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";

  oss << "radar_measurments[number: " << radar_objects_.size() << ",";
  for (auto it = radar_objects_.begin(); it != radar_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first
        << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";

  oss << "camera_measurments[number: " << camera_objects_.size() << ",";
  for (auto it = camera_objects_.begin(); it != camera_objects_.end(); ++it) {
    oss << "(sensor_id: " << it->first
        << ", invisible_t: " << it->second->GetInvisiblePeriod()
        << ", info: " << it->second->GetBaseObject()->ToString() << ")\n";
  }
  oss << "]\n";
  return oss.str();
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
