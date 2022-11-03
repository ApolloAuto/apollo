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
#include "modules/task_manager/proto/task_manager_config.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"

namespace apollo {
namespace task_manager {

class ParkingRoutingManager {
 public:
  ParkingRoutingManager();
  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(
      const task_manager::ParkingRoutingTask& parking_routing_task);

  bool ConstructParkingRoutingRequest(ParkingRoutingTask* parking_routing_task);

  /**
   * @brief destructor
   */
  virtual ~ParkingRoutingManager() = default;

 private:
  bool has_space_ = false;
  bool has_space_id_ = false;
  std::string id_ = "";
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

}  // namespace task_manager
}  // namespace apollo
