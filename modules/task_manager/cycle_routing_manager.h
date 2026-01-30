/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include <string>
#include <vector>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/task_manager_msgs/task_manager.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/geometry.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"

namespace apollo {
namespace task_manager {

class CycleRoutingManager {
 public:
  CycleRoutingManager() = default;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const localization::Pose& pose, const task_manager::CycleRoutingTask& cycle_routing_task);

  /**
   * @brief Get new routing if the vehicle reaches the begin/end point
   * @return false/true
   */
  bool GetNewRouting(const localization::Pose& pose,
                     external_command::LaneFollowCommand* lane_follow_command);

  /**
   * @brief get remaining cycle number
   * @return remaining cycle number
   */
  int GetCycle() const { return cycle_; }

  /**
   * @brief destructor
   */
  virtual ~CycleRoutingManager() = default;

 private:
  int cycle_ = 0;
  int waypoint_num_ = 0;
  bool is_allowed_to_route_ = false;
  external_command::Pose begin_point_;
  external_command::Pose end_point_;
  std::unique_ptr<apollo::dreamview::MapService> map_service_;
  external_command::LaneFollowCommand original_lane_follow_command_;
};

}  // namespace task_manager
}  // namespace apollo
