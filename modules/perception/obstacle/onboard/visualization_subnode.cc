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

#include "modules/perception/obstacle/onboard/visualization_subnode.h"

#include <string>
#include <unordered_map>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/onboard/dag_streaming.h"

#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {
namespace lowcostvisualizer {

using apollo::common::ErrorCode;
using apollo::common::Status;

bool VisualizationSubnode::InitInternal() {
  CHECK(shared_data_manager_ != NULL);
  // init stream
  if (!InitStream()) {
    AERROR << "Failed to init stream.";
    return false;
  }

  // init camera object data
  if (camera_event_id_ != -1) {
    camera_object_data_ = dynamic_cast<CameraObjectData*>(
        shared_data_manager_->GetSharedData("CameraObjectData"));
    if (camera_object_data_ == nullptr) {
      AERROR << "Failed to get CameraObjectData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << camera_object_data_->name();

    camera_shared_data_ = dynamic_cast<CameraSharedData*>(
        shared_data_manager_->GetSharedData("CameraSharedData"));
    if (camera_shared_data_ == nullptr) {
      AERROR << "Failed to get CameraSharedData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << camera_shared_data_->name();
  }
  // init cipv object data
  if (cipv_event_id_ != -1) {
    cipv_object_data_ = dynamic_cast<CIPVObjectData*>(
          shared_data_manager_->GetSharedData("CIPVObjectData"));
    if (cipv_object_data_ == nullptr) {
            AERROR << "Failed to get CIPVObjectData.";
            return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << cipv_object_data_->name();
  }
  //  init radar object data
  if (radar_event_id_ != -1) {
    radar_object_data_ = dynamic_cast<RadarObjectData*>(
        shared_data_manager_->GetSharedData("RadarObjectData"));
    if (radar_object_data_ == nullptr) {
      AERROR << "Failed to get RadarObjectData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << radar_object_data_->name();
  }

  // init fusion data
  if (fusion_event_id_ != -1) {
    fusion_data_ = dynamic_cast<FusionSharedData*>(
        shared_data_manager_->GetSharedData("FusionSharedData"));
    if (fusion_data_ == nullptr) {
      AERROR << "Failed to get FusionSharedData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: " << fusion_data_->name();
  }

  // init motion service
  if (motion_event_id_ != -1) {
      motion_service_ = dynamic_cast<MotionService*>(
        DAGStreaming::GetSubnodeByName("MotionService"));
    if (motion_service_ == nullptr) {
      AERROR << "motion service not inited";
      return false;
    }
  }

  if (lane_event_id_ != -1) {
    lane_shared_data_ = dynamic_cast<LaneSharedData*>(
        shared_data_manager_->GetSharedData("LaneSharedData"));
    if (lane_shared_data_ == nullptr) {
      AERROR << "Failed to get LaneSharedData.";
      return false;
    }
    AINFO << "Init shared data successfully, data: "
          << lane_shared_data_->name();
  }

  // init frame_visualizer
  RegisterFactoryGLFusionVisualizer();
  frame_visualizer_.reset(
      BaseVisualizerRegisterer::GetInstanceByName(FLAGS_frame_visualizer));
  if (!frame_visualizer_) {
    AERROR << "Failed to get instance: " << FLAGS_frame_visualizer;
    return false;
  }
  content_.set_pose_type(FrameContent::IMAGE_CONTINUOUS);
  AINFO << "visualize according to continuous image: ";

  CalibrationConfigManager* config_manager =
      Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
  camera_to_car_pose_ = calibrator->get_camera_extrinsics();
  AINFO << "Init camera to car transform successfully.";
  content_.set_camera2car_pose(camera_to_car_pose_);
  return true;
}

bool VisualizationSubnode::InitStream() {
  std::unordered_map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve string: " << reserve_;
    return false;
  }

  auto iter = reserve_field_map.find("vis_driven_event_id");
  if (iter == reserve_field_map.end()) {
    AERROR << "Failed to find vis_driven_event_id:" << reserve_;
    return false;
  }
  vis_driven_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));

  iter = reserve_field_map.find("camera_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find camera_event_id_: " << reserve_;
    camera_event_id_ = -1;
  } else {
    camera_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("radar_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find radar_event_id_: " << reserve_;
    radar_event_id_ = -1;
  } else {
    radar_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("fusion_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find fusion_event_id_: " << reserve_;
    fusion_event_id_ = -1;
  } else {
    fusion_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("cipv_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find cipv_event_id_: " << reserve_;
    cipv_event_id_ = -1;
  } else {
    cipv_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("motion_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find motion_event_id_: " << reserve_;
    motion_event_id_ = -1;
  } else {
    motion_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  iter = reserve_field_map.find("lane_event_id");
  if (iter == reserve_field_map.end()) {
    AWARN << "Failed to find lane_event_id_: " << reserve_;
    lane_event_id_ = -1;
  } else {
    lane_event_id_ = static_cast<EventID>(atoi((iter->second).c_str()));
  }

  return true;
}

bool VisualizationSubnode::SubscribeEvents(const EventMeta& event_meta,
                                           std::vector<Event>* events) const {
  Event event;
  if (event_meta.event_id == vis_driven_event_id_) {
    event_manager_->Subscribe(event_meta.event_id, &event);
    events->push_back(event);
  } else {
    // no blocking
    while (event_manager_->Subscribe(event_meta.event_id, &event, true)) {
      events->push_back(event);
    }
  }

  return true;
}

void VisualizationSubnode::SetFrameContent(const Event& event,
                                           const std::string& device_id,
                                           const std::string& data_key,
                                           const double timestamp,
                                           FrameContent* content) {
  if (event.event_id == camera_event_id_) {
    std::shared_ptr<CameraItem> camera_item;
    if (!camera_shared_data_->Get(data_key, &camera_item) ||
        camera_item == nullptr) {
      AERROR << "Failed to get shared data: " << camera_shared_data_->name();
      return;
    }
    cv::Mat image = camera_item->image_src_mat.clone();
    content->set_image_content(timestamp, image);

    std::shared_ptr<SensorObjects> objs;
    if (!camera_object_data_->Get(data_key, &objs) || objs == nullptr) {
      AERROR << "Failed to get shared data: " << camera_object_data_->name();
      return;
    }
    content->set_camera_content(timestamp, objs->sensor2world_pose,
                                objs->objects,
                                (*(objs->camera_frame_supplement)));
  } else if (event.event_id == motion_event_id_) {
//    AINFO << "Vis_subnode: motion_event_id_" << motion_event_id_;
    // TODO(gchen-apollo): add lock to read motion_buffer
    MotionBufferPtr motion_buffer = motion_service_->GetMotionBuffer();
    if (motion_buffer == nullptr) {
      AINFO << "motion_buffer is null";
    } else {
      content->set_motion_content(timestamp, motion_buffer);
    }
  } else if (event.event_id == radar_event_id_) {
    if (device_id == "radar_front" && FLAGS_show_radar_objects) {
      std::shared_ptr<SensorObjects> objs;
      if (!radar_object_data_->Get(data_key, &objs) || objs == nullptr) {
        AERROR << "Failed to get shared data: " << radar_object_data_->name();
        return;
      }
      content->set_radar_content(timestamp, objs->objects);
    }
  } else if (event.event_id == fusion_event_id_) {
    bool show_fused_objects = true;
    if (show_fused_objects) {
      AINFO << "vis_driven_event data_key = " << data_key;
      SharedDataPtr<FusionItem> fusion_item;
      if (!fusion_data_->Get(data_key, &fusion_item) ||
          fusion_item == nullptr) {
        AERROR << "Failed to get shared data: " << fusion_data_->name();
        return;
      }
      content->set_fusion_content(timestamp, fusion_item->obstacles);

      AINFO << "Set fused objects : " << fusion_item->obstacles.size();
    }
  } else if (event.event_id == cipv_event_id_) {
    if (FLAGS_show_camera_objects || FLAGS_show_camera_objects2d ||
        FLAGS_show_camera_parsing) {
      std::shared_ptr<CameraItem> camera_item;
      if (!camera_shared_data_->Get(data_key, &camera_item) ||
          camera_item == nullptr) {
        AERROR << "Failed to get shared data: " << camera_shared_data_->name();
        return;
      }
      cv::Mat clone_image = camera_item->image_src_mat;
      cv::Mat image = camera_item->image_src_mat.clone();
      content->set_image_content(timestamp, image);

      std::shared_ptr<SensorObjects> objs;
      if (!cipv_object_data_->Get(data_key, &objs) || objs == nullptr) {
        AERROR << "Failed to get shared data: " << cipv_object_data_->name();
        return;
      }

      LOG(INFO) << "number of objects in cipv is " << objs->objects.size()
                << timestamp << " with cipv index is " << objs->cipv_index;

      //   content->set_camera2velo_pose(_camera_to_velo64_pose);

      if (FLAGS_show_camera_parsing) {
        // content->set_camera_content(timestamp, objs->sensor2world_pose,
        //                            objs->objects,
        //                            (*(objs->camera_frame_supplement)));
      } else {
        content->set_camera_content(timestamp, objs->sensor2world_pose,
                                    objs->objects);
      }
    }
  } else if (event.event_id == lane_event_id_) {
    LaneObjectsPtr lane_objs;
    if (!lane_shared_data_->Get(data_key, &lane_objs) || lane_objs == nullptr) {
      AERROR << "Failed to get shared data: " << lane_shared_data_->name();
      return;
    }
    content->set_lane_content(timestamp, *lane_objs);
  }

  if (event.event_id == vis_driven_event_id_) {
    content->update_timestamp(timestamp);
  }
}

apollo::common::Status VisualizationSubnode::ProcEvents() {
  for (auto event_meta : sub_meta_events_) {
//    AINFO <<"Vis_sub: event_meta id: " << event_meta.event_id;
    std::vector<Event> events;
    if (!SubscribeEvents(event_meta, &events)) {
      return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
    }
    if (events.empty()) continue;

    for (size_t j = 0; j < events.size(); j++) {
      double timestamp = events[j].timestamp;
      const std::string& device_id = events[j].reserve;
      std::string data_key;

      if (!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id,
                                               &data_key)) {
        AERROR << "Failed to produce shared data key. timestamp:" << timestamp
               << " device_id:" << device_id;
        return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
      }
      AINFO << "event: " << events[j].event_id << " device_id:" << device_id
            << " timestamp: ";
      AINFO << std::fixed << std::setprecision(64) << timestamp;

      SetFrameContent(events[j], device_id, data_key, timestamp, &content_);
    }

    if (event_meta.event_id == vis_driven_event_id_) {
      // Init of frame_visualizer must be in one thread with render,
      if (!init_) {
        frame_visualizer_->init();
        init_ = true;
      }
      frame_visualizer_->update_camera_system(&content_);
      frame_visualizer_->render(&content_);
    }
  }
  return Status::OK();
}

}  // namespace lowcostvisualizer
}  // namespace perception
}  // namespace apollo
