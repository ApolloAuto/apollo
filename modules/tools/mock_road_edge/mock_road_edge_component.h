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

#pragma once

#include <memory>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"
#include "modules/planning/planning_base/proto/learning_data.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/component/timer_component.h"
#include "cyber/message/raw_message.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/time/clock.h"
#include "modules/planning/planning_base/common/message_process.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::perception::PerceptionEdgeInfo;

class MockRoadEdgeComponent final : public apollo::cyber::TimerComponent {
 public:
  MockRoadEdgeComponent() = default;

  ~MockRoadEdgeComponent() = default;

 public:
  bool Init() override;

  bool Proc() override;

 private:
  bool CheckInput();
  void MockEdgeInfo(hdmap::RouteSegments* truncated_segments);
  bool InJunction(const PointENU& check_point, const double radius);

 private:
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<PlanningCommand>> planning_command_reader_;
  std::shared_ptr<cyber::Writer<PerceptionEdgeInfo>> mock_road_edge_writer_;

  std::mutex mutex_;
  routing::RoutingResponse routing_;
  PlanningCommand planning_command_;
  PlanningCommand last_command_;
  apollo::localization::LocalizationEstimate localization_;
  apollo::canbus::Chassis chassis_;
  LocalView local_view_;
  common::VehicleState vehicle_state_;
  apollo::common::VehicleStateProvider vehicle_state_provider_;

  PlanningConfig config_;
  std::shared_ptr<planning::PncMapBase> current_pnc_map_;
  apollo::hdmap::LaneWaypoint adc_waypoint_;
  PerceptionEdgeInfo edge_info_;
};

CYBER_REGISTER_COMPONENT(MockRoadEdgeComponent)

}  // namespace planning
}  // namespace apollo
