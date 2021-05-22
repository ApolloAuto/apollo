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

#include "modules/localization/proto/localization.pb.h"
#include "modules/task_manager/proto/task_manager.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/dreamview/backend/map/map_service.h"

namespace apollo {
namespace task_manager {

class CycleRoutingManager {
 public:
  CycleRoutingManager() = default;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const task_manager::CycleRoutingTask& cycle_routing_task);

  /**
   * @brief Get new routing if the vehicle reaches the begin/end point
   * @return false/true
   */
  bool GetNewRouting(const localization::Pose& pose,
                     routing::RoutingRequest* routing_request_);

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
  routing::LaneWaypoint begin_point_;
  routing::LaneWaypoint end_point_;
  std::unique_ptr<apollo::dreamview::MapService> map_service_;
  routing::RoutingRequest original_routing_request_;
};

}  // namespace task_manager
}  // namespace apollo
