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
#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_FUSION_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_FUSION_SUBNODE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.h"
#include "modules/perception/obstacle/fusion/async_fusion/async_fusion.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/onboard/fusion_shared_data.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/fusion/interface/base_fusion.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {

class FusionSubnode : public Subnode {
 public:
  FusionSubnode() = default;
  virtual ~FusionSubnode() {}
  apollo::common::Status ProcEvents() override;
  bool GeneratePbMsg(PerceptionObstacles *obstacles);

 protected:
  bool InitInternal() override;

 private:
  bool InitOutputStream();
  bool SubscribeEvents(const EventMeta &event_meta,
                       std::vector<Event> *events) const;
  bool BuildSensorObjs(const std::vector<Event> &events,
                       std::vector<SensorObjects> *multi_sensor_objs);
  bool ProducePbMsg(double timestamp, SeqId seq_num,
                    const std::vector<std::shared_ptr<Object>> &fused_objs,
                    std::string *pb_msg) const;
  bool GetSharedData(const Event &event,
                     std::shared_ptr<SensorObjects> *sensor_objects);
  apollo::common::Status Process(const EventMeta &event_meta,
                                 const std::vector<Event> &events);
  void RegistAllAlgorithm();

  void OnChassis(const apollo::canbus::Chassis &message);

  void PublishDataAndEvent(const double &timestamp,
                           const std::string &device_id,
                           const SharedDataPtr<FusionItem> &data);
  double timestamp_;
  std::vector<std::shared_ptr<Object>> objects_;
  common::ErrorCode error_code_ = common::OK;
  std::unique_ptr<BaseFusion> fusion_;
  LidarObjectData *lidar_object_data_ = nullptr;
  RadarObjectData *radar_object_data_ = nullptr;
  CameraObjectData *camera_object_data_ = nullptr;
  FusionSharedData *fusion_data_ = nullptr;
  LaneSharedData *lane_shared_data_ = nullptr;
  std::shared_ptr<LaneObjects> lane_objects_;
  // lidar perception subnode event controls the publishing behavior
  EventID pub_driven_event_id_;
  EventID lidar_event_id_;
  EventID radar_event_id_;
  EventID camera_event_id_;
  EventID lane_event_id_;
  std::mutex fusion_subnode_mutex_;
  apollo::canbus::Chassis chassis_;
  volatile float chassis_speed_mps_;
  DISALLOW_COPY_AND_ASSIGN(FusionSubnode);
};

REGISTER_SUBNODE(FusionSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_FUSION_SUBNODE_H_
