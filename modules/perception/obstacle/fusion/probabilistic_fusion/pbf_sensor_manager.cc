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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_manager.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

/*
PbfSensorManager *PbfSensorManager::Instance() {
static PbfSensorManager sensor_manager;
return &sensor_manager;
}
*/

PbfSensorManager::PbfSensorManager() {}

PbfSensorManager::~PbfSensorManager() {
  for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
    if (it->second != nullptr) {
      delete (it->second);
      it->second = nullptr;
    }
  }
  sensors_.clear();
}

bool PbfSensorManager::Init() {
  sensors_.clear();

  std::string sensor_id = GetSensorType(SensorType::VELODYNE_64);
  SensorType type = SensorType::VELODYNE_64;

  PbfSensor *velodyne_64 = new PbfSensor(sensor_id, type);
  sensors_[sensor_id] = velodyne_64;

  sensor_id = GetSensorType(SensorType::RADAR);
  type = SensorType::RADAR;
  PbfSensor *radar = new PbfSensor(sensor_id, type);
  if (radar == nullptr) {
    AERROR << "Fail to create PbfSensor. sensor_id = " << sensor_id;
    return false;
  }
  if (sensors_.find(sensor_id) == sensors_.end()) {
    sensors_[sensor_id] = radar;
  } else {
    AERROR << "The velodyne 64 and radar sensor ids are conflict.";
    delete radar;
    return false;
  }
  return true;
}

void PbfSensorManager::AddSensorMeasurements(const SensorObjects &objects) {
  std::string sensor_id = GetSensorType(objects.sensor_type);
  SensorType type = objects.sensor_type;

  auto it = sensors_.find(sensor_id);
  PbfSensor *sensor = nullptr;
  if (it == sensors_.end()) {
    AWARN << "Cannot find sensor " << sensor_id
          << " and create one in SensorManager";
    sensor = new PbfSensor(sensor_id, type);
    if (sensor == nullptr) {
      AERROR << "Fail to create PbfSensor. sensor_id = " << sensor_id;
      return;
    }
    sensors_[sensor_id] = sensor;
  } else {
    sensor = it->second;
  }
  sensor->AddFrame(objects);
}

void PbfSensorManager::GetLatestSensorFrames(
    const double time_stamp, const std::string &sensor_id,
    std::vector<PbfSensorFramePtr> *frames) {
  if (frames == nullptr) {
    return;
  }
  auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    return;
  }
  return it->second->QueryLatestFrames(time_stamp, frames);
}

void PbfSensorManager::GetLatestFrames(const double time_stamp,
                                       std::vector<PbfSensorFramePtr> *frames) {
  if (frames == nullptr) {
    return;
  }
  frames->clear();
  for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
    PbfSensor *sensor = it->second;
    PbfSensorFramePtr frame = sensor->QueryLatestFrame(time_stamp);
    if (frame != nullptr) {
      frames->push_back(frame);
    }
  }
  std::sort(frames->begin(), frames->end(),
            [](const PbfSensorFramePtr &p1, const PbfSensorFramePtr &p2) {
              return p1->timestamp < p2->timestamp;
            });
}

PbfSensor *PbfSensorManager::GetSensor(const std::string &sensor_id) {
  std::map<std::string, PbfSensor *>::iterator it = sensors_.find(sensor_id);
  PbfSensor *sensor = nullptr;
  if (it != sensors_.end()) {
    sensor = it->second;
  }
  return sensor;
}

bool PbfSensorManager::GetPose(const std::string &sensor_id, double time_stamp,
                               const double time_range, Eigen::Matrix4d *pose) {
  if (pose == nullptr) {
    AERROR << "output parameter pose is nullptr";
    return false;
  }

  std::map<std::string, PbfSensor *>::iterator it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    AERROR << "Failed to find sensor for " << sensor_id;
    return false;
  }

  PbfSensor *sensor = it->second;
  return sensor->GetPose(time_stamp, time_range, pose);
}

}  // namespace perception
}  // namespace apollo
