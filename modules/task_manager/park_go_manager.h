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

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/task_manager/proto/task_manager.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include"modules/canbus/proto/chassis.pb.h"
namespace apollo {
namespace task_manager {
using apollo::routing::RoutingRequest;
using apollo::localization::LocalizationEstimate;
using apollo::routing::LaneWaypoint;
class ParkGoManager {
 public:
  ParkGoManager();
  /**
   * @brief module initialization function
   * @return initialization status
   */
common::Status Init(const ParkGOTask& park_go_task);
RoutingRequest generate(LocalizationEstimate& localization,int index ,std::string lane_id,double s);
  
bool near(LocalizationEstimate& localization,int index);
  virtual ~ParkGoManager() = default;

 private:
  double stay_time_;
  bool update_;
  int current_stage_;
  std::string id_ = "";
  std::unique_ptr<std::thread> execute_thread_ = nullptr;
  std::vector<LaneWaypoint> wp_list_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>> localization_reader_;

  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> request_writer_;
};

}  // namespace task_manager
}  // namespace apollo
