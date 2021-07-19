/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/task_manager/proto/task_manager.pb.h"
#include "modules/task_manager/proto/task_manager_config.pb.h"
#include "modules/task_manager/common/task_manager_gflags.h"

namespace apollo {
namespace task_manager {

class DeadEndRoutingManager {
 public:
  DeadEndRoutingManager();
  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const task_manager::DeadEndRoutingTask&
                      dead_end_routing_task);
  /**
   * @brief destructor
   */
  virtual ~DeadEndRoutingManager() = default;

  int GetNumber() const { return cycle_; }

  bool GetNewRouting(const localization::Pose& pose,
                     routing::RoutingRequest* routing_request);

  bool JudgeCarInDeadEndJunction(const common::math::Vec2d& car_position,
                                 const common::PointENU& target_point);

 private:
  int cycle_ = 0;
  bool routing_in_flag_ = true;
  bool routing_out_flag_ = true;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  routing::RoutingRequest routing_request_in_;
  routing::RoutingRequest routing_request_out_;
};

}  // namespace task_manager
}  // namespace apollo
