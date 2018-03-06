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

#include <vector>
#include <string>
#include <map>
#include "modules/common/log.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

using apollo::common::ErrorCode;
using apollo::common::Status;

bool VisualizationSubnode::InitInternal() {
  CHECK(shared_data_manager_ != NULL);
  // init radar object data
  if (FLAGS_show_radar_objects) {
    _radar_object_data = dynamic_cast<RadarObjectData*>(
        shared_data_manager_->GetSharedData("RadarObjectData"));
    if (_radar_object_data == nullptr) {
      AERROR << "Failed to get RadarObjectData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << _radar_object_data->name();
  }

  // init camera object data
  if (FLAGS_show_camera_objects || FLAGS_show_camera_objects2d ||
      FLAGS_show_camera_parsing) {
    _camera_object_data = dynamic_cast<CameraObjectData*>(
        shared_data_manager_->GetSharedData("CameraObjectData"));
    if (_camera_object_data == nullptr) {
      AERROR << "Failed to get CameraObjectData.";
      return false;
    }
    _cipv_object_data = dynamic_cast<CIPVObjectData*>(
        shared_data_manager_->GetSharedData("CIPVObjectData"));
    if (_cipv_object_data == nullptr) {
      AERROR << "Failed to get CIPVObjectData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << _camera_object_data->name();
  }

  // init fusion data
  if (FLAGS_show_fused_objects) {
    _fusion_data = dynamic_cast<FusionSharedData*>(
        shared_data_manager_->GetSharedData("FusionSharedData"));
    if (_fusion_data == nullptr) {
      AERROR << "Failed to get FusionSharedDataData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: " << _fusion_data->name();
  }

  // init camera shared data
  if (FLAGS_show_camera_objects || FLAGS_show_camera_objects2d ||
      FLAGS_show_camera_parsing) {
    _camera_shared_data = dynamic_cast<CameraSharedData*>(
        shared_data_manager_->GetSharedData("CameraSharedData"));
    if (_camera_shared_data == nullptr) {
      AERROR << "Failed to get CameraSharedData.";
      return false;
    }
    AINFO << "Init shared datas successfully, data: "
          << _camera_shared_data->name();
  }
  // init frame_visualizer
  _frame_visualizer.reset(
      BaseVisualizerRegisterer::GetInstanceByName(FLAGS_frame_visualizer));
  if (!_frame_visualizer) {
    AERROR << "Failed to get instance: " << FLAGS_frame_visualizer;
    return false;
  }
  _content.set_pose_type(FrameContent::IMAGE_CONTINUOUS);
  AINFO << "visualize according to continuous image: ";
  // init stream
  if (!InitStream()) {
    AERROR << "Failed to init stream.";
    return false;
  }

  CalibrationConfigManager* config_manager =
      Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
  _camera_to_car_pose = calibrator->get_camera_extrinsics();
  AINFO << "Init camera to car transform successfully.";
  _content.set_camera2car_pose(_camera_to_car_pose);
  return true;
}

bool VisualizationSubnode::InitStream() {
  std::map<std::string, std::string> reserve_field_map;
  if (!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map)) {
    AERROR << "Failed to parse reserve string: " << reserve_;
    return false;
  }

  auto iter = reserve_field_map.find("vis_driven_event_id");
  if (iter == reserve_field_map.end()) {
    AERROR << "Failed to find vis_driven_event_id:" << reserve_;
    return false;
  }
  _vis_driven_event_id = static_cast<EventID>(atoi((iter->second).c_str()));

  auto radar_iter = reserve_field_map.find("radar_event_id");
  if (radar_iter == reserve_field_map.end()) {
    AERROR << "Failed to find radar_event_id:" << reserve_;
    return false;
  }
  _radar_event_id = static_cast<EventID>(atoi((radar_iter->second).c_str()));

  auto camera_iter = reserve_field_map.find("camera_event_id");
  if (camera_iter == reserve_field_map.end()) {
    AERROR << "Failed to find camera_event_id:" << reserve_;
    return false;
  }
  _camera_event_id = static_cast<EventID>(atoi((camera_iter->second).c_str()));

  auto cipv_iter = reserve_field_map.find("cipv_event_id");
  if (cipv_iter == reserve_field_map.end()) {
    AERROR << "Failed to find cipv_event_id:" << reserve_;
    return false;
  }
  _cipv_event_id = static_cast<EventID>(atoi((cipv_iter->second).c_str()));

  auto fusion_iter = reserve_field_map.find("fusion_event_id");
  if (fusion_iter == reserve_field_map.end()) {
    AERROR << "Failed to find fusion_event_id:" << reserve_;
    return false;
  }
  _fusion_event_id = static_cast<EventID>(atoi((fusion_iter->second).c_str()));

  auto motion_iter = reserve_field_map.find("motion_event_id");
  if (motion_iter == reserve_field_map.end()) {
    AERROR << "Failed to find motion_event_id:" << reserve_;
    _motion_event_id = -1;
  } else {
    _motion_event_id =
        static_cast<EventID>(atoi((motion_iter->second).c_str()));
  }

  return true;
}

bool VisualizationSubnode::SubscribeEvents(const EventMeta& event_meta,
                                           std::vector<Event>* events) const {
  Event event;
  if (event_meta.event_id == _vis_driven_event_id) {
    // blocking
    //        if (!event_manager_->subscribe(event_meta.event_id, &event, true))
    //        {
    //            // AERROR << "Failed to subscribe event: " <<
    //            event_meta.event_id;
    //            // return false;
    //        }
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

void VisualizationSubnode::GetFrameData(const Event& event,
                                        const std::string& device_id,
                                        const std::string& data_key,
                                        const double timestamp,
                                        FrameContent* content) {
  if (event.event_id == _camera_event_id) {
    if (FLAGS_show_camera_objects || FLAGS_show_camera_objects2d ||
        FLAGS_show_camera_parsing) {
      std::shared_ptr<CameraItem> camera_item;
      if (!_camera_shared_data->Get(data_key, &camera_item) ||
          camera_item == nullptr) {
        AERROR << "Failed to get shared data: " << _camera_shared_data->name();
        return;
      }
      cv::Mat clone_image = camera_item->image_src_mat;
      cv::Mat image = camera_item->image_src_mat.clone();
      content->set_image_content(timestamp, image);

      std::shared_ptr<SensorObjects> objs;
      if (!_camera_object_data->Get(data_key, &objs) || objs == nullptr) {
        AERROR << "Failed to get shared data: " << _camera_object_data->name();
        return;
      }

      LOG(INFO) << objs->objects.size() << timestamp;

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
  } else if (event.event_id == _motion_event_id) {
    /*std::shared_ptr<CameraItem> camera_item;
    AERROR << "Motion_Visualization key: in Motion Visualization: " << data_key;
    if (!_camera_shared_data->Get(data_key, &camera_item) ||
        camera_item == nullptr) {
      AERROR << "Failed to get shared data in Motion Visualization: "
             << _camera_shared_data->name() << " " << data_key;
      return;
    }*/

    // content->set_motion_content(timestamp, camera_item->motion_buffer);

    //        std::cout<< "motion_buffer.size(): " <<
    //        camera_item->motion_buffer->size() << std::endl;

  } else if (event.event_id == _radar_event_id) {
    if (device_id == "radar_front" && FLAGS_show_radar_objects) {
      std::shared_ptr<SensorObjects> objs;
      if (!_radar_object_data->Get(data_key, &objs) || objs == nullptr) {
        AERROR << "Failed to get shared data: " << _radar_object_data->name();
        return;
      }
      content->set_radar_content(timestamp, objs->objects);
    }
  } else if (event.event_id == _fusion_event_id) {
    if (FLAGS_show_fused_objects) {
      AINFO << "vis_driven_event data_key = " << data_key;
      SharedDataPtr<FusionItem> fusion_item;
      if (!_fusion_data->Get(data_key, &fusion_item) ||
          fusion_item == nullptr) {
        AERROR << "Failed to get shared data: " << _fusion_data->name();
        return;
      }
      content->set_fusion_content(timestamp, fusion_item->obstacles);

      AINFO << "Set fused objects : " << fusion_item->obstacles.size();
    }
  } else if (event.event_id == _cipv_event_id) {
    if (FLAGS_show_camera_objects || FLAGS_show_camera_objects2d ||
        FLAGS_show_camera_parsing) {
      std::shared_ptr<CameraItem> camera_item;
      if (!_camera_shared_data->Get(data_key, &camera_item) ||
          camera_item == nullptr) {
        AERROR << "Failed to get shared data: " << _camera_shared_data->name();
        return;
      }
      cv::Mat clone_image = camera_item->image_src_mat;
      cv::Mat image = camera_item->image_src_mat.clone();
      content->set_image_content(timestamp, image);

      std::shared_ptr<SensorObjects> objs;
      if (!_cipv_object_data->Get(data_key, &objs) || objs == nullptr) {
        AERROR << "Failed to get shared data: " << _cipv_object_data->name();
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
  }

  if (event.event_id == _vis_driven_event_id) {
    // _vis_driven_event_id fusion -> visualization
    content->update_timestamp(timestamp);
  }
}

apollo::common::Status VisualizationSubnode::ProcEvents() {
  for (auto event_meta : sub_meta_events_) {
    std::vector<Event> events;
    if (!SubscribeEvents(event_meta, &events)) {
      return Status(ErrorCode::PERCEPTION_ERROR, "Failed to proc events.");
    }
    if (events.empty()) {
      continue;
    }
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
      //            GetFrameData(device_id, data_key, &_content, timestamp);
      if (event_meta.event_id == _vis_driven_event_id) {
        AERROR << "vis_driven_event_1: " << events[j].event_id << " "
               << timestamp << " " << device_id << " " << _motion_event_id;
      }
      GetFrameData(events[j], device_id, data_key, timestamp, &_content);
      if (event_meta.event_id == _vis_driven_event_id) {
        // Init of frame_visualizer must be in one thread with render,
        // so you must move it from init_internal.
        if (!_init) {
          _frame_visualizer->init();
          // if (_camera_visualizer) {
          //     _camera_visualizer->init();
          // }
          _init = true;
        }
        _frame_visualizer->update_camera_system(&_content);
        _frame_visualizer->render(&_content);
        //                _frame_visualizer->set_motion_buffer(_motion_buffer);
        // if (_camera_visualizer) {
        //     _camera_visualizer->render(_content);
        // }
      }
    }
  }
  return Status::OK();
}

REGISTER_SUBNODE(VisualizationSubnode);

}  // namespace perception
}  // namespace apollo
