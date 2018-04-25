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

// @brief: lane_post_processing_subnode source file
#include "modules/perception/obstacle/onboard/lane_post_processing_subnode.h"

#include <chrono>
#include <thread>
#include <algorithm>
#include <cfloat>
#include <unordered_map>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/cc_lane_post_processor.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/types.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::time::Timer;
using std::shared_ptr;
using std::string;
using std::unordered_map;

const int MAX_MOTION_SERVICE_DELAY = 5;
bool LanePostProcessingSubnode::InitInternal() {
  // get Subnode config in DAG streaming
  unordered_map<string, string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);
  if (fields.count("publish") && stoi(fields["publish"]) != 0) {
    publish_ = true;
  }
  auto iter = fields.find("motion_event_id");
  if (iter == fields.end()) {
    motion_event_id_ = -1;
    AWARN << "Failed to find motion_event_id_:" << reserve_;
    AWARN << "Unable to project lane history information";
  } else {
    motion_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
    motion_service_ = dynamic_cast<MotionService *>(
        DAGStreaming::GetSubnodeByName("MotionService"));
    if (motion_service_ == nullptr) {
      AWARN << "motion service should initialize before LanePostProcessing";
    }
    options_.use_lane_history = true;
    AINFO << "options_.use_lane_history: " << options_.use_lane_history;
  }
  // init shared data
  if (!InitSharedData()) {
    AERROR << "failed to init shared data.";
    return false;
  }

  RegistAllAlgorithms();

  // init plugins
  if (!InitAlgorithmPlugin()) {
    AERROR << "failed to init algorithm plugins.";
    return false;
  }

  AINFO << "init LanePostProcessing subnode successfully.";
  return true;
}

bool LanePostProcessingSubnode::InitSharedData() {
  if (shared_data_manager_ == nullptr) {
    AERROR << "shared data manager is a null pointer.";
    return false;
  }

  // init preprocess_data
  camera_object_data_ = dynamic_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  if (camera_object_data_ == nullptr) {
    AERROR << "failed to get shared data instance: CameraObjectData ";
    return false;
  }
  lane_shared_data_ = dynamic_cast<LaneSharedData *>(
      shared_data_manager_->GetSharedData("LaneSharedData"));
  if (lane_shared_data_ == nullptr) {
    AERROR << "failed to get shared data instance: LaneSharedData ";
    return false;
  }

  AINFO << "init shared data successfully, data: "
        << camera_object_data_->name() << " and " << lane_shared_data_->name();

  return true;
}

void LanePostProcessingSubnode::RegistAllAlgorithms() {
  RegisterFactoryCCLanePostProcessor();
}

bool LanePostProcessingSubnode::InitAlgorithmPlugin() {
  // init lane post-processer
  lane_post_processor_.reset(
      BaseCameraLanePostProcessorRegisterer::GetInstanceByName(
          FLAGS_onboard_lane_post_processor));
  if (!lane_post_processor_) {
    AERROR << "failed to get instance: " << FLAGS_onboard_lane_post_processor;
    return false;
  }
  if (!lane_post_processor_->Init()) {
    AERROR << "failed to init lane post-processor: "
           << lane_post_processor_->name();
    return false;
  }

  AINFO << "init alg pulgins successfully\n"
        << " lane post-processer:     " << FLAGS_onboard_lane_post_processor;
  return true;
}

bool LanePostProcessingSubnode::InitWorkRoot() {
  ConfigManager *config_manager = ConfigManager::instance();
  if (config_manager == NULL) {
    AERROR << "failed to get ConfigManager instance.";
    return false;
  }

  if (!config_manager->Init()) {
    AERROR << "failed to init ConfigManager";
    return false;
  }

  return true;
}

bool LanePostProcessingSubnode::GetSharedData(const Event &event,
                                              shared_ptr<SensorObjects> *objs) {
  double timestamp = event.timestamp;
  string device_id = event.reserve;
  device_id_ = device_id;
  string data_key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id, &data_key)) {
    AERROR << "failed to produce shared data key. EventID:" << event.event_id
           << " timestamp:" << timestamp << " device_id:" << device_id;
    return false;
  }

  if (!camera_object_data_->Get(data_key, objs)) {
    AERROR << "failed to get shared data. event:" << event.to_string();
    return false;
  }
  return true;
}

void LanePostProcessingSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<LaneObjects> &lane_objects) {
  string key;
  if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id_, &key)) {
    AERROR << "failed to produce shared key. time: "
           << GLOG_TIMESTAMP(timestamp) << ", device_id: " << device_id_;
    return;
  }

  if (!lane_shared_data_->Add(key, lane_objects)) {
    AWARN << "failed to add LaneSharedData. key: " << key
          << " num_detected_objects: " << lane_objects->size();
    return;
  }

  // pub events
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
  ADEBUG << "succeed to publish data and event.";
}

Status LanePostProcessingSubnode::ProcEvents() {
  // fusion output subnode only subcribe the fusion subnode
  CHECK_EQ(sub_meta_events_.size(), 1u) << "only subcribe one event.";
  const EventMeta &event_meta = sub_meta_events_[0];
  Event event;
  event_manager_->Subscribe(event_meta.event_id, &event);
  ++seq_num_;
  shared_ptr<SensorObjects> objs;
  if (!GetSharedData(event, &objs)) {
    AERROR << "Failed to get shared data. event:" << event.to_string();
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
  }

  Timer timer;
  timer.Start();

  cv::Mat lane_map = objs->camera_frame_supplement->lane_map;
  if (lane_map.empty()) {
    AERROR << "Get NULL lane_map from camera frame supplement";
    return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
  }

  LaneObjectsPtr lane_objects(new LaneObjects());
  options_.timestamp = event.timestamp;
  timestamp_ns_ = event.timestamp * 1e9;
  if (motion_event_id_ != -1) {
    if (motion_service_ == nullptr) {
      motion_service_ = dynamic_cast<MotionService *>(
          DAGStreaming::GetSubnodeByName("MotionService"));
      if (motion_service_ == nullptr) {
        AERROR << "motion service must initialize before LanePostProcessing";
        return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
      }
    }

    double motion_timestamp = motion_service_->GetLatestTimestamp();
    ADEBUG << "object ts : motion ts   " << std::to_string(event.timestamp)
          << "  " << std::to_string(motion_timestamp);

    if (motion_timestamp > event.timestamp) {
      if (!motion_service_->GetMotionInformation(
          event.timestamp, &(options_.vehicle_status))) {
        AERROR << "cannot find desired motion in motion buffer at: "
               << std::to_string(event.timestamp);
        options_.vehicle_status.time_ts = 0.0;  // signal to reset history
        // return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
      }
    } else if (motion_timestamp < event.timestamp) {
      int count = 0;
      while (motion_timestamp < event.timestamp) {
        count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ADEBUG << "delay in motion: " << count;
        ADEBUG << "object ts : motion ts  " << std::to_string(event.timestamp)
              << "  " << std::to_string(motion_timestamp);
        motion_timestamp = motion_service_->GetLatestTimestamp();
        // exceed max waiting time
        if (motion_timestamp > 0 && count > MAX_MOTION_SERVICE_DELAY) {
          break;
        }
      }
      mutex_.lock();
      options_.SetMotion(motion_service_->GetMotionBuffer()->back());
      mutex_.unlock();
      if (event.timestamp - options_.vehicle_status.time_ts > 0.2) {
          options_.vehicle_status.time_ts = 0.0;  // signal to reset history
      }
    } else {
      mutex_.lock();
      options_.SetMotion(motion_service_->GetMotionBuffer()->back());
      mutex_.unlock();
    }
    ADEBUG << "options_.vehicle_status.motion:  "
          << options_.vehicle_status.motion;
  }
  lane_post_processor_->Process(lane_map, options_, &lane_objects);
  for (size_t i = 0; i < lane_objects->size(); ++i) {
    (*lane_objects)[i].timestamp = event.timestamp;
    (*lane_objects)[i].seq_num = seq_num_;
  }
  ADEBUG << "Before publish lane objects, objects num: "
         << lane_objects->size();

  uint64_t t = timer.End("lane post-processing");
  min_processing_time_ = std::min(min_processing_time_, t);
  max_processing_time_ = std::max(max_processing_time_, t);
  tot_processing_time_ += t;
  ADEBUG << "Lane Post Processing Runtime: "
         << "MIN (" << min_processing_time_ << " ms), "
         << "MAX (" << max_processing_time_ << " ms), "
         << "AVE (" << tot_processing_time_ / seq_num_ << " ms).";

  PublishDataAndEvent(event.timestamp, lane_objects);

  if (publish_) {
    PublishPerceptionPb(lane_objects);
  }

  ADEBUG << "Successfully finished lane post processing";
  return Status::OK();
}

void LanePostProcessingSubnode::PublishPerceptionPb(
    const LaneObjectsPtr &lane_objects) {
  ADEBUG << "Lane post-processor publish lane object pb data";

  PerceptionObstacles obstacles;

  // Header
  common::adapter::AdapterManager::FillPerceptionObstaclesHeader(
      "perception_obstacle", &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(timestamp_ns_);
  header->set_radar_timestamp(0);

  // generate lane marker protobuf messages
  LaneMarkers *lane_markers = obstacles.mutable_lane_marker();
  LaneObjectsToLaneMarkerProto(*lane_objects, lane_markers);

  common::adapter::AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "Lane Markers: " << obstacles.ShortDebugString();

  ADEBUG << "Succeed to publish lane object pb data.";
}

}  // namespace perception
}  // namespace apollo
