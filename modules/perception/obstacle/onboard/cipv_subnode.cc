
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

#include "modules/perception/obstacle/onboard/cipv_subnode.h"

#include <vector>
#include <string>
#include <map>
#include <memory>
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/onboard/dag_streaming.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

using apollo::common::ErrorCode;
using apollo::common::Status;

using std::vector;
using std::string;
using std::map;
// using Event;
// using EventID;
// using EventMeta;
// using IoStreamType;
// using SharedDataPtr;
// using SubnodeHelper;
// using Status;
// using StreamOutput;
// using std_msgs::String;

bool CIPVSubnode::InitInternal() {
    CHECK(shared_data_manager_ != nullptr);
    // init camera object data
    camera_object_data_ = dynamic_cast<CameraObjectData*>(
        shared_data_manager_->GetSharedData("CameraObjectData"));
    if (camera_object_data_ == nullptr) {
      AERROR << "Failed to get CameraObjectData.";
      return false;
    }
    // init camera object data
    lane_shared_data_ = dynamic_cast<LaneSharedData*>(
        shared_data_manager_->GetSharedData("LaneSharedData"));
    if (lane_shared_data_ == nullptr) {
      AERROR << "Failed to get LaneSharedData.";
      return false;
    }

    cipv_object_data_ = dynamic_cast<CIPVObjectData*>(
            shared_data_manager_->GetSharedData("CIPVObjectData"));
    if (cipv_object_data_ == nullptr) {
        AERROR << "Failed to get CIPVObjectData";
        return false;
    }

    AINFO << "Init shared datas successfully";

    string reserve_;
    map<string, string> reserve_field_map;
    if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
        AERROR << "Failed to parse reserve string: " << reserve_;
        return false;
    }

    if (!InitOutputStream(reserve_field_map)) {
        AERROR << "Failed to init output or input stream.";
        return false;
    }
    // init Cipv
    cipv_.Init();

    AINFO << "Init CIPVSubnode succ.";
    return true;
}

apollo::common::Status CIPVSubnode::ProcEvents() {
    Event event;
    if (!SubscribeEvents(&event)) {
        return Status(ErrorCode::PERCEPTION_ERROR,
                      "Failed to subscribe events.");
    }

    std::shared_ptr<SensorObjects> sensor_objs;
    if (!GetSharedData(event, &sensor_objs)) {
        return Status(ErrorCode::PERCEPTION_ERROR,
                      "Failed to get shared data.");
    }
    CipvOptions cipv_options;
    // *** To Do *** use motion manager
    // MotionService* motion_service =
    //   dynamic_cast<MotionService*>
    //     (DAGStreaming::GetSubnodeByName("MotionService"));
    // double timestamp = event.timestamp;
    // VehicleInformation vehicle_information;
    // motion_service->get_vehicle_information(timestamp,
    //                                                &vehicle_information);
    // cipv_options.yaw_angle = vehicle_information.yaw_angle;
    // cipv_options.velocity = vehicle_information.velocity;
    // cipv_options.yaw_rate = vehicle_information.yaw_rate;
    cipv_options.yaw_angle = 0.0f;  // ***** fill in the value *****
    cipv_options.velocity = 5.0f;  // ***** fill in the value *****
    cipv_options.yaw_rate = 0.0f;  // ***** fill in the value *****
    AINFO << "[CIPVSubnode] velocity " << cipv_options.velocity
              << ", yaw rate: " << cipv_options.yaw_rate
              << ", yaw angle: " << cipv_options.yaw_angle;

    // call cipv module
    if (cipv_.DetermineCipv(sensor_objs, &cipv_options)) {
      // *** To Do *** when camera_detector_subnode.cpp is included, add this
      // publish_data_and_event(event.timestamp, event.reserve,
      //                        sensor_objs, cipv_object_data_);
    }

    return Status::OK();
}

bool CIPVSubnode::InitOutputStream(const map<string, string>& fields) {
    auto camera_iter = fields.find("camera_event_id");
    if (camera_iter == fields.end()) {
       AERROR << "Failed to find camera_event_id";
       camera_event_id_ = -1;
    } else {
       camera_event_id_ =
         static_cast<EventID>(atoi((camera_iter->second).c_str()));
       AINFO << "camera event id is " << camera_event_id_;
    }

    auto lane_iter = fields.find("lane_event_id");
    if (lane_iter == fields.end()) {
       AERROR << "Failed to find lane_event_id";
       lane_event_id_ = -1;
    } else {
       lane_event_id_ =
         static_cast<EventID>(atoi((lane_iter->second).c_str()));
       AINFO << "lane event id is " << lane_event_id_;
    }

    AINFO << "Init output stream succ";
    return true;
}

bool CIPVSubnode::SubscribeEvents(Event *event) const {
    if (!event_manager_->Subscribe(camera_event_id_, event)) {
        AERROR << "Failed to subscribe event: " << camera_event_id_;
        return false;
    }
    if (!event_manager_->Subscribe(lane_event_id_, event)) {
        AERROR << "Failed to subscribe event: " << lane_event_id_;
        return false;
    }
    return true;
}

bool CIPVSubnode::GetSharedData(const Event& event,
                                std::shared_ptr<SensorObjects>* objs) const {
  double timestamp = event.timestamp;
  const string& device_id = event.reserve;
  string data_key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id,
                                              &data_key)) {
    AERROR << "Failed to produce shared data key. EventID:" << event.event_id
           << " timestamp:" << timestamp << " device_id:" << device_id;
    return false;
  }
  bool get_data_succ = false;
  // *** To DO *** uncomment
  get_data_succ = camera_object_data_->Get(data_key, objs);
  std::shared_ptr<LaneObjects> lane_objects;
  get_data_succ = lane_shared_data_->Get(data_key, &lane_objects);
  (*objs)->lane_objects = lane_objects;

  if (!get_data_succ) {
    AERROR << "Failed to get shared data. event:" << event.to_string();
    return false;
  }
  return true;
}

REGISTER_SUBNODE(CIPVSubnode);

}  // namespace perception
}  // namespace apollo
