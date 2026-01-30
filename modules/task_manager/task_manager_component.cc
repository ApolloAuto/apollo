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
#include "modules/task_manager/task_manager_component.h"

#include "modules/task_manager/proto/task_manager_config.pb.h"

#include "cyber/time/rate.h"

namespace apollo {
namespace task_manager {

using apollo::cyber::ComponentBase;
using apollo::cyber::Rate;
using apollo::external_command::CommandStatus;
using apollo::external_command::LaneFollowCommand;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::planning::PlanningCommand;
using apollo::routing::RoutingResponse;

bool TaskManagerComponent::Init() {
  TaskManagerConfig task_manager_conf;
  ACHECK(cyber::ComponentBase::GetProtoConfig(&task_manager_conf))
      << "Unable to load task_manager conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      task_manager_conf.topic_config().localization_pose_topic(),
      [this](const std::shared_ptr<LocalizationEstimate>& localization) {
        ADEBUG << "Received localization data: run localization callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        localization_.CopyFrom(*localization);
      });

  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      task_manager_conf.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        ADEBUG << "Received planning_command: run response callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });

  lane_follow_command_client_ =
      node_->CreateClient<LaneFollowCommand, CommandStatus>(
          task_manager_conf.topic_config().lane_follow_command_topic());
  return true;
}

bool TaskManagerComponent::Proc(const std::shared_ptr<Task>& task) {
  if (task->task_type() != CYCLE_ROUTING) {
    AERROR << "Task type is not cycle_routing.";
    return false;
  }

  if (task->task_type() == CYCLE_ROUTING) {
    cycle_routing_manager_ = std::make_shared<CycleRoutingManager>();
    cycle_routing_manager_->Init(localization_.pose(), task->cycle_routing_task());
    lane_follow_command_ = task->cycle_routing_task().lane_follow_command();
    Rate rate(1.0);

    while (cycle_routing_manager_->GetCycle() > 0) {
      if (cycle_routing_manager_->GetNewRouting(localization_.pose(),
                                                &lane_follow_command_)) {
        auto last_planning_command_ = planning_command_;
        common::util::FillHeader(node_->Name(), &lane_follow_command_);
        auto lane_follow_command = std::make_shared<LaneFollowCommand>();
        lane_follow_command->CopyFrom(lane_follow_command_);
        lane_follow_command_client_->SendRequest(lane_follow_command);
        AINFO << "[TaskManagerComponent]Reach begin/end point: "
              << "routing manager send a routing request. ";
        rate.Sleep();

        if (!planning_command_.has_header()) {
          AINFO << "[TaskManagerComponent]routing failed";
          return false;
        }
        if (last_planning_command_.has_header()) {
          if (last_planning_command_.header().sequence_num() ==
              planning_command_.header().sequence_num()) {
            AINFO << "[TaskManagerComponent]No routing response: "
                  << "new routing failed";
            return false;
          }
        }
      }
      rate.Sleep();
    }
  }
  return true;
}

}  // namespace task_manager
}  // namespace apollo
