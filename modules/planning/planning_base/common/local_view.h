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
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/planning_msgs/navigation.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"
#include "modules/common_msgs/control_msgs/control_interactive_msg.pb.h"

namespace apollo {
namespace planning {

/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<relative_map::MapMsg> relative_map;
  std::shared_ptr<PadMessage> pad_msg;
  std::shared_ptr<storytelling::Stories> stories;
  std::shared_ptr<PlanningCommand> planning_command;
  std::shared_ptr<routing::LaneWaypoint> end_lane_way_point;
  std::shared_ptr<control::ControlInteractiveMsg> control_interactive_msg;
  std::shared_ptr<perception::PerceptionAccurateDockInfo> perception_dock;
};

}  // namespace planning
}  // namespace apollo
