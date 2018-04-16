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
#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_CIPV_SUBNODE_H
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_CIPV_SUBNODE_H

#include <memory>
#include <string>
#include <unordered_map>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/perception/obstacle/camera/cipv/cipv.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

class CIPVSubnode : public Subnode {
 public:
  CIPVSubnode() = default;
  virtual ~CIPVSubnode() {}
  apollo::common::Status ProcEvents() override;

 protected:
  bool InitInternal() override;

 private:
  bool InitOutputStream(
      const std::unordered_map<std::string, std::string>& fields);
  bool SubscribeEvents(Event* event) const;

  bool GetSharedData(const Event& event,
                     std::shared_ptr<SensorObjects>* sensor_objects);
  void PublishDataAndEvent(const float& timestamp,
                           const SharedDataPtr<SensorObjects>& sensor_objects,
                           CIPVObjectData* cipv_object_data);

  CameraObjectData* camera_object_data_ = nullptr;
  LaneSharedData* lane_shared_data_ = nullptr;
  CIPVObjectData* cipv_object_data_ = nullptr;
  EventID camera_event_id_;
  EventID lane_event_id_;
  Cipv cipv_;
  std::string device_id_;
  DISALLOW_COPY_AND_ASSIGN(CIPVSubnode);
};

REGISTER_SUBNODE(CIPVSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_CIPV_SUBNODE_H
