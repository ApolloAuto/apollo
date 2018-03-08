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

#include <map>
#include <memory>
#include <string>
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/obstacle/camera/cipv/cipv.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

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
    bool InitOutputStream(const std::map<std::string, std::string>& fields);
    bool SubscribeEvents(Event* event) const;

    bool GetSharedData(
            const Event& event,
            std::shared_ptr<SensorObjects>* sensor_objects) const;

    CameraObjectData* _camera_object_data = nullptr;
    LaneSharedData* _lane_shared_data = nullptr;
    CIPVObjectData* _cipv_object_data = nullptr;
    EventID _camera_event_id;
    EventID _lane_event_id;
    Cipv _cipv;
    DISALLOW_COPY_AND_ASSIGN(CIPVSubnode);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_CIPV_SUBNODE_H
