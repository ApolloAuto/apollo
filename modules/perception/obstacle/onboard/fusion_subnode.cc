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

#include "modules/perception/obstacle/onboard/fusion_subnode.h"

#include <unordered_map>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;

bool FusionSubnode::InitInternal() {
  RegistAllAlgorithm();

  AdapterManager::Init(FLAGS_perception_adapter_config_filename);

  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  AdapterManager::AddChassisCallback(&FusionSubnode::OnChassis, this);

  CHECK(shared_data_manager_ != nullptr);
  fusion_.reset(BaseFusionRegisterer::GetInstanceByName(FLAGS_onboard_fusion));
  if (fusion_ == nullptr) {
    AERROR << "Failed to get fusion instance: " << FLAGS_onboard_fusion;
    return false;
  }
  if (!fusion_->Init()) {
    AERROR << "Failed to init fusion:" << FLAGS_onboard_fusion;
    return false;
  }
  radar_object_data_ = dynamic_cast<RadarObjectData *>(
      shared_data_manager_->GetSharedData("RadarObjectData"));
  if (radar_object_data_ == nullptr) {
    AWARN << "Failed to get RadarObjectData.";
  }
  lidar_object_data_ = dynamic_cast<LidarObjectData *>(
      shared_data_manager_->GetSharedData("LidarObjectData"));
  if (lidar_object_data_ == nullptr) {
    AWARN << "Failed to get LidarObjectData.";
  }
  camera_object_data_ = dynamic_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  if (camera_object_data_ == nullptr) {
    AWARN << "Failed to get CameraObjectData.";
  }

  fusion_data_ = dynamic_cast<FusionSharedData *>(
      shared_data_manager_->GetSharedData("FusionSharedData"));
  if (fusion_data_ == nullptr) {
    AWARN << "Failed to get FusionSharedData.";
  }

  lane_shared_data_ = dynamic_cast<LaneSharedData *>(
      shared_data_manager_->GetSharedData("LaneSharedData"));
  if (lane_shared_data_ == nullptr) {
    AWARN << "failed to get shared data instance: LaneSharedData ";
  }

  lane_objects_.reset(new LaneObjects());

  if (!InitOutputStream()) {
    AERROR << "Failed to init output stream.";
    return false;
  }

  AINFO << "Init FusionSubnode succ. Using fusion:" << fusion_->name();
  return true;
}

bool FusionSubnode::InitOutputStream() {
  // expect reserve_ format:
  //       pub_driven_event_id:n
  //       lidar_output_stream : event_id=n&sink_type=m&sink_name=x
  //       radar_output_stream : event_id=n&sink_type=m&sink_name=x
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve string: " << reserve_;
    return false;
  }
  auto iter = reserve_field_map.find("pub_driven_event_id");
  if (iter == reserve_field_map.end()) {
    AERROR << "Failed to find pub_driven_event_id:" << reserve_;
    return false;
  }
  pub_driven_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));

  auto lidar_iter = reserve_field_map.find("lidar_event_id");
  if (lidar_iter == reserve_field_map.end()) {
    AWARN << "Failed to find lidar_event_id:" << reserve_;
    AINFO << "lidar_event_id will be set -1";
    lidar_event_id_ = -1;
  } else {
    lidar_event_id_ = static_cast<EventID>(atoi((lidar_iter->second).c_str()));
  }

  auto radar_iter = reserve_field_map.find("radar_event_id");
  if (radar_iter == reserve_field_map.end()) {
    AWARN << "Failed to find radar_event_id:" << reserve_;
    AINFO << "radar_event_id will be set -1";
    radar_event_id_ = -1;
  } else {
    radar_event_id_ = static_cast<EventID>(atoi((radar_iter->second).c_str()));
  }

  auto camera_iter = reserve_field_map.find("camera_event_id");
  if (camera_iter == reserve_field_map.end()) {
    AWARN << "Failed to find camera_event_id:" << reserve_;
    AINFO << "camera_event_id will be set -1";
    camera_event_id_ = -1;
  } else {
    camera_event_id_ =
        static_cast<EventID>(atoi((camera_iter->second).c_str()));
  }

  auto lane_iter = reserve_field_map.find("lane_event_id");
  if (lane_iter == reserve_field_map.end()) {
    AWARN << "Failed to find lane_event_id_:" << reserve_;
    AINFO << "lane_event_id_ will be set -1";
    lane_event_id_ = -1;
  } else {
    lane_event_id_ = static_cast<EventID>(atoi((lane_iter->second).c_str()));
  }

  return true;
}

Status FusionSubnode::ProcEvents() {
  for (auto event_meta : sub_meta_events_) {
    if (event_meta.event_id == lane_event_id_) continue;  // ignore lane event

    std::vector<Event> events;
    if (!SubscribeEvents(event_meta, &events)) {
      AERROR << "event meta id:" << event_meta.event_id << " "
             << event_meta.from_node << " " << event_meta.to_node;
      return Status(ErrorCode::PERCEPTION_ERROR, "Subscribe event fail.");
    }
    if (events.empty()) {
      usleep(500);
      continue;
    }
    Process(event_meta, events);

    if (event_meta.event_id != pub_driven_event_id_) {
      ADEBUG << "Not pub_driven_event_id, skip to publish."
             << " event_id:" << event_meta.event_id
             << " fused_obj_cnt:" << objects_.size();
      continue;
    }

    // public obstacle message
    PerceptionObstacles obstacles;
    if (GeneratePbMsg(&obstacles)) {
      common::adapter::AdapterManager::PublishPerceptionObstacles(obstacles);
    }
    AINFO << "Publish 3d perception fused msg. timestamp:"
          << GLOG_TIMESTAMP(timestamp_) << " obj_cnt:" << objects_.size();
  }
  return Status::OK();
}

Status FusionSubnode::Process(const EventMeta &event_meta,
                              const std::vector<Event> &events) {
  std::vector<SensorObjects> sensor_objs;
  if (!BuildSensorObjs(events, &sensor_objs)) {
    AERROR << "Failed to build_sensor_objs";
    error_code_ = common::PERCEPTION_ERROR_PROCESS;
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to build_sensor_objs.");
  }
  PERF_BLOCK_START();
  objects_.clear();
  if (!fusion_->Fuse(sensor_objs, &objects_)) {
    AWARN << "Failed to call fusion plugin."
          << " event_meta: [" << event_meta.to_string()
          << "] event_cnt:" << events.size() << " event_0: ["
          << events[0].to_string() << "]";
    error_code_ = common::PERCEPTION_ERROR_PROCESS;
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to call fusion plugin.");
  }
  if (event_meta.event_id == lidar_event_id_) {
    PERF_BLOCK_END("fusion_lidar");
  } else if (event_meta.event_id == radar_event_id_) {
    PERF_BLOCK_END("fusion_radar");
  } else if (event_meta.event_id == camera_event_id_) {
    for (auto &obj : sensor_objs) {
      if (obj.sensor_type == SensorType::CAMERA) {
        AINFO << "fusion received image : " << GLOG_TIMESTAMP(obj.timestamp)
              << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
        break;
      }
    }
    PERF_BLOCK_END("fusion_camera");
  }

  if (objects_.size() > 0 && FLAGS_publish_fusion_event) {
    SharedDataPtr<FusionItem> fusion_item_ptr(new FusionItem);
    fusion_item_ptr->timestamp = objects_[0]->latest_tracked_time;
    const std::string &device_id = events[0].reserve;
    for (auto obj : objects_) {
      std::shared_ptr<Object> objclone(new Object());
      objclone->clone(*obj);
      fusion_item_ptr->obstacles.push_back(objclone);
    }
    AINFO << "publishing event for timestamp deviceid and size of fusion object"
          << fusion_item_ptr->timestamp << " " << device_id << " "
          << fusion_item_ptr->obstacles.size();
    PublishDataAndEvent(fusion_item_ptr->timestamp, device_id, fusion_item_ptr);
  }

  timestamp_ = sensor_objs[0].timestamp;
  error_code_ = common::OK;
  return Status::OK();
}

void FusionSubnode::PublishDataAndEvent(const double &timestamp,
                                        const std::string &device_id,
                                        const SharedDataPtr<FusionItem> &data) {
  CommonSharedDataKey key(timestamp, device_id);
  bool fusion_succ = fusion_data_->Add(key, data);
  if (!fusion_succ) {
    AERROR << "fusion shared data addkey failure";
  }
  AINFO << "adding key in fusion shared data " << key.ToString();

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id;
    event_manager_->Publish(event);
  }
}

bool FusionSubnode::SubscribeEvents(const EventMeta &event_meta,
                                    std::vector<Event> *events) const {
  Event event;
  // no blocking
  while (event_manager_->Subscribe(event_meta.event_id, &event, true)) {
    events->push_back(event);
  }
  return true;
}
bool FusionSubnode::BuildSensorObjs(
    const std::vector<Event> &events,
    std::vector<SensorObjects> *multi_sensor_objs) {
  PERF_FUNCTION();
  for (auto event : events) {
    std::shared_ptr<SensorObjects> sensor_objects;
    if (!GetSharedData(event, &sensor_objects)) {
      return false;
    }
    // Make sure timestamp and type are filled.
    sensor_objects->timestamp = event.timestamp;
    if (event.event_id == lidar_event_id_) {
      sensor_objects->sensor_type = SensorType::VELODYNE_64;
    } else if (event.event_id == radar_event_id_) {
      sensor_objects->sensor_type = SensorType::RADAR;
    } else if (event.event_id == camera_event_id_) {
      sensor_objects->sensor_type = SensorType::CAMERA;
    } else {
      AERROR << "Event id is not supported. event:" << event.to_string();
      return false;
    }
    sensor_objects->sensor_id = GetSensorType(sensor_objects->sensor_type);
    multi_sensor_objs->push_back(*sensor_objects);
    ADEBUG << "get sensor objs:" << sensor_objects->ToString();
  }
  return true;
}

bool FusionSubnode::GetSharedData(const Event &event,
                                  std::shared_ptr<SensorObjects> *objs) {
  double timestamp = event.timestamp;
  const std::string &device_id = event.reserve;
  std::string data_key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id, &data_key)) {
    AERROR << "Failed to produce shared data key. EventID:" << event.event_id
           << " timestamp:" << timestamp << " device_id:" << device_id;
    return false;
  }
  bool get_data_succ = false;
  if (event.event_id == lidar_event_id_ && lidar_object_data_ != nullptr) {
    get_data_succ = lidar_object_data_->Get(data_key, objs);
  } else if (event.event_id == radar_event_id_ &&
             radar_object_data_ != nullptr) {
    get_data_succ = radar_object_data_->Get(data_key, objs);
  } else if (event.event_id == camera_event_id_ &&
             camera_object_data_ != nullptr) {
    get_data_succ = camera_object_data_->Get(data_key, objs);

    if (lane_shared_data_ != nullptr && lane_event_id_ != -1) {
      Event lane_event;
      if (event_manager_->Subscribe(lane_event_id_, &lane_event, false)) {
        get_data_succ = lane_shared_data_->Get(data_key, &lane_objects_);
        ADEBUG << "getting lane data successfully for data key " << data_key;
      }
    }
  } else {
    AERROR << "Event id is not supported. event:" << event.to_string();
    return false;
  }
  if (!get_data_succ) {
    AERROR << "Failed to get shared data. event:" << event.to_string();
    return false;
  }
  return true;
}

bool FusionSubnode::GeneratePbMsg(PerceptionObstacles *obstacles) {
  common::adapter::AdapterManager::FillPerceptionObstaclesHeader(
      FLAGS_obstacle_module_name, obstacles);
  common::Header *header = obstacles->mutable_header();
  if (pub_driven_event_id_ == lidar_event_id_) {
    header->set_lidar_timestamp(timestamp_ * 1e9);  // in ns
    header->set_camera_timestamp(0);
    header->set_radar_timestamp(0);
  } else if (pub_driven_event_id_ == camera_event_id_) {
    header->set_lidar_timestamp(0);  // in ns
    header->set_camera_timestamp(timestamp_ * 1e9);
    header->set_radar_timestamp(0);
  }

  obstacles->set_error_code(error_code_);

  for (const auto &obj : objects_) {
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    obj->Serialize(obstacle);
  }

  if (FLAGS_use_navigation_mode) {
    // Relative speed of objects + latest ego car speed in X
    for (auto obstacle : obstacles->perception_obstacle()) {
      obstacle.mutable_velocity()->set_x(obstacle.velocity().x() +
                                         chassis_speed_mps_);
    }

    // generate lane marker protobuf messages
    if (lane_shared_data_ != nullptr && lane_event_id_ != -1) {
      LaneMarkers *lane_markers = obstacles->mutable_lane_marker();
      LaneObjectsToLaneMarkerProto(*(lane_objects_), lane_markers);
    }
  }

  ADEBUG << "PerceptionObstacles: " << obstacles->ShortDebugString();
  return true;
}

void FusionSubnode::RegistAllAlgorithm() {
  RegisterFactoryProbabilisticFusion();
  RegisterFactoryAsyncFusion();
}

void FusionSubnode::OnChassis(const Chassis &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  chassis_.CopyFrom(chassis);
  ADEBUG << "Received chassis information " << chassis_.ShortDebugString();
  chassis_speed_mps_ = chassis_.speed_mps();
}

}  // namespace perception
}  // namespace apollo
