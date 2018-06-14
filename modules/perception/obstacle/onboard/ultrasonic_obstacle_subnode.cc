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

#include "modules/perception/obstacle/onboard/ultrasonic_obstacle_subnode.h"

#include <cmath>
#include <utility>
#include <unordered_map>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/timer.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::VehicleStateProvider;

bool UltrasonicObstacleSubnode::InitInternal() {
  if (!InitAlgorithmPlugin()) {
        AERROR << "Failed to init algorithm plugin.";
        return false;
    }
  // parse reserve fileds
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve filed: " << reserve_;
    return false;
  }

  if (reserve_field_map.find("device_id") == reserve_field_map.end()) {
    AERROR << "Failed to find field device_id, reserve: " << reserve_;
    return false;
  }
  device_id_ = reserve_field_map["device_id"];

  CHECK(AdapterManager::GetChassis()) << "Failed to get Ultrasonic adapter";
  AdapterManager::AddChassisCallback(
      &UltrasonicObstacleSubnode::OnUltrasonic, this);

  ADEBUG << "Succeed to finish ultrasonic detector initialization!";

  return true;
}

void UltrasonicObstacleSubnode::OnUltrasonic(
    const apollo::canbus::Chassis& message) {
  ++seq_num_;
  PERF_BLOCK_START();
  std::shared_ptr<SensorObjects> sensor_objects(new SensorObjects);
  double timestamp = message.header().timestamp_sec();
  sensor_objects->timestamp = timestamp;
  sensor_objects->sensor_type = SensorType::ULTRASONIC;
  sensor_objects->sensor_id = device_id_;
  sensor_objects->seq_num = seq_num_;

  BuildAllObjects(message.surround(), sensor_objects);

  if (!PublishDataAndEvent(timestamp, sensor_objects)) {
    AERROR << "Failed to publish data.";
    sensor_objects->error_code = apollo::common::PERCEPTION_ERROR_PROCESS;
    return;
  }
  ADEBUG << "ultrasonic object size: " << sensor_objects->objects.size();
  PERF_BLOCK_END("ultrasonic_detect");
}

bool UltrasonicObstacleSubnode::InitAlgorithmPlugin() {
    /// init share data
    CHECK(shared_data_manager_ != nullptr);
    // init preprocess_data
    const std::string processing_data_name("UltrasonicObjectData");
    processing_data_ = dynamic_cast<UltrasonicObjectData*>(
        shared_data_manager_->GetSharedData(processing_data_name));
    if (processing_data_ == nullptr) {
      AERROR << "Failed to get shared data instance "
             << processing_data_name;
      return false;
    }
    ADEBUG << "Init shared data successfully, data: "
           << processing_data_->name();
    return true;
}

bool UltrasonicObstacleSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<SensorObjects>& data) {
  std::string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "Failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return false;
  }

  processing_data_->Add(key, data);

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta& event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }

  return true;
}

void UltrasonicObstacleSubnode::BuildSingleObject(
    const apollo::canbus::Sonar& sonar,
    std::shared_ptr<Object> object_ptr) {
  object_ptr->track_id = 0;
  object_ptr->type = ObjectType::UNKNOWN;
  object_ptr->velocity = {0.0, 0.0, 0.0};
  double vehicle_x = VehicleStateProvider::instance()->x();
  double vehicle_y = VehicleStateProvider::instance()->y();
  double vehicle_z = VehicleStateProvider::instance()->z();
  double vehicle_heading = VehicleStateProvider::instance()->heading();
  double sonar_x = vehicle_x + sonar.translation().x();
  double sonar_y = vehicle_y + sonar.translation().y();
  double sonar_z = vehicle_z + sonar.translation().z();
  double sonar_relative_heading = apollo::common::math::QuaternionToHeading(
      sonar.rotation().qw(), sonar.rotation().qx(),
      sonar.rotation().qy(), sonar.rotation().qz());
  double sonar_heading = vehicle_heading + sonar_relative_heading;
  double sonar_obs_x = sonar_x + sonar.range() * std::cos(sonar_heading);
  double sonar_obs_y = sonar_y + sonar.range() * std::cos(sonar_heading);
  double half_width = 0.2;  // TODO(kechxu) refactor
  double length = 0.2;  // TODO(kechxu) refactor
  double alpha = sonar_heading - M_PI / 2.0;
  std::vector<std::pair<double, double>> vertices;
  double near_left_x = sonar_obs_x - half_width * half_width * std::cos(alpha);
  double near_left_y = sonar_obs_y - half_width * half_width * std::sin(alpha);
  double near_right_x = sonar_obs_x + half_width * half_width * std::cos(alpha);
  double near_right_y = sonar_obs_y + half_width * half_width * std::sin(alpha);
  vertices.emplace_back(near_left_x, near_left_y);
  vertices.emplace_back(near_right_x, near_right_y);
  vertices.emplace_back(
      near_right_x + length * std::cos(sonar_heading),
      near_right_y + length * std::sin(sonar_heading));
  vertices.emplace_back(
      near_left_x + length * std::cos(sonar_heading),
      near_left_y + length * std::sin(sonar_heading));

  auto& polygon = object_ptr->polygon;
  polygon.resize(vertices.size());
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    polygon.points[i].x = vertices[i].first;
    polygon.points[i].y = vertices[i].second;
    polygon.points[i].z = sonar_z;
  }
  CHECK_GT(polygon.points.size(), 0);
  object_ptr->theta = sonar_heading;
  Eigen::Vector3d direction(std::cos(sonar_heading),
                            std::sin(sonar_heading), 0.0);
  object_ptr->direction = direction;
  object_ptr->length = length;
  object_ptr->width = 2.0 * half_width;
  object_ptr->height = sonar_z;
  double anchor_x = 0.0;
  double anchor_y = 0.0;
  for (size_t i = 0; i < polygon.points.size(); ++i) {
    anchor_x += polygon.points[i].x;
    anchor_y += polygon.points[i].y;
  }
  anchor_x /= static_cast<double>(polygon.points.size());
  anchor_y /= static_cast<double>(polygon.points.size());
  double anchor_z = 0.5 * sonar_z;
  Eigen::Vector3d anchor_point(anchor_x, anchor_y, anchor_z);
  object_ptr->anchor_point = anchor_point;
  // TODO(kechxu) object_ptr->score_type
}

void UltrasonicObstacleSubnode::BuildAllObjects(
    const apollo::canbus::Surround& surround,
    std::shared_ptr<SensorObjects> sensor_objects) {
  for (const auto& sonar : surround.sonar()) {
    std::shared_ptr<Object> object_ptr(new Object);
    BuildSingleObject(sonar, object_ptr);
    sensor_objects->objects.push_back(std::move(object_ptr));
  }
}

}  // namespace perception
}  // namespace apollo
