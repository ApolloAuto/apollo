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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_VISUALIZATION_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_VISUALIZATION_SUBNODE_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "modules/perception/obstacle/camera/visualizer/base_visualizer.h"
#include "modules/perception/obstacle/camera/visualizer/frame_content.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/fusion_shared_data.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

class VisualizationSubnode : public Subnode {
 public:
  VisualizationSubnode() = default;

  // @breif: c++ spec ask for explicit destructor because of pointer class
  // member
  virtual ~VisualizationSubnode() {}

  bool InitInternal() override;

  apollo::common::Status ProcEvents() override;

 private:
  bool InitStream();
  bool SubscribeEvents(const EventMeta& event_meta,
                       std::vector<Event>* events) const;

  // void get_frame_data(const std::string& device_id,
  //                     const std::string& data_key,
  //                     FrameContent* content, double timestamp);
  void GetFrameData(const Event& event, const std::string& device_id,
                    const std::string& data_key, const double timestamp,
                    FrameContent* content);

  RadarObjectData* _radar_object_data = nullptr;
  CameraObjectData* _camera_object_data = nullptr;
  CIPVObjectData* _cipv_object_data = nullptr;
  CameraSharedData* _camera_shared_data = nullptr;
  FusionSharedData* _fusion_data = nullptr;
  std::unique_ptr<BaseVisualizer> _frame_visualizer;
  FrameContent _content;

  EventID _vis_driven_event_id;
  EventID _radar_event_id;
  EventID _camera_event_id;
  EventID _fusion_event_id;
  EventID _motion_event_id;
  EventID _cipv_event_id;

  //    MotionBufferPtr _motion_buffer;
  Eigen::Matrix4d _camera_to_car_pose;

  bool _init = false;
  DISALLOW_COPY_AND_ASSIGN(VisualizationSubnode);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_VISUALIZATION_SUBNODE_H
