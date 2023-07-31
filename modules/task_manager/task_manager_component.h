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

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/task_manager/cycle_routing_manager.h"
#include "modules/task_manager/parking_routing_manager.h"

namespace apollo {
namespace task_manager {

using apollo::localization::LocalizationEstimate;

class TaskManagerComponent final : public cyber::Component<task_manager::Task> {
 public:
  TaskManagerComponent() = default;
  ~TaskManagerComponent() = default;

 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<task_manager::Task>& task) override;

 private:
  std::shared_ptr<cyber::Reader<LocalizationEstimate>> localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::PlanningCommand>>
      planning_command_reader_;
  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> trajectory_reader_;
  std::shared_ptr<cyber::Client<apollo::external_command::LaneFollowCommand,
                                apollo::external_command::CommandStatus>>
      lane_follow_command_client_;
  std::shared_ptr<CycleRoutingManager> cycle_routing_manager_;
  std::shared_ptr<ParkingRoutingManager> parking_routing_manager_;
  external_command::LaneFollowCommand lane_follow_command_;
  planning::PlanningCommand planning_command_;
  LocalizationEstimate localization_;
  apollo::planning::ADCTrajectory planning_;
  std::mutex mutex_;
  std::string task_name_;
};

CYBER_REGISTER_COMPONENT(TaskManagerComponent)

}  // namespace task_manager
}  // namespace apollo
