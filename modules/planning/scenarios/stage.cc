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

/**
 * @file
 **/

#include "modules/planning/scenarios/stage.h"

#include <unordered_map>
#include <utility>

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::common::time::Clock;

namespace {
// constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
// constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

Stage::Stage(const ScenarioConfig::StageConfig& config) : config_(config) {
  // set stage_type in PlanningContext
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(stage_type());

  name_ = ScenarioConfig::StageType_Name(config_.stage_type());
  next_stage_ = config_.stage_type();
  std::unordered_map<TaskConfig::TaskType, const TaskConfig*, std::hash<int>>
      config_map;
  for (const auto& task_config : config_.task_config()) {
    config_map[task_config.task_type()] = &task_config;
  }
  for (int i = 0; i < config_.task_type_size(); ++i) {
    auto task_type = config_.task_type(i);
    ACHECK(config_map.find(task_type) != config_map.end())
        << "Task: " << TaskConfig::TaskType_Name(task_type)
        << " used but not configured";
    auto iter = tasks_.find(task_type);
    if (iter == tasks_.end()) {
      auto ptr = TaskFactory::CreateTask(*config_map[task_type]);
      task_list_.push_back(ptr.get());
      tasks_[task_type] = std::move(ptr);
    } else {
      task_list_.push_back(iter->second.get());
    }
  }
}

const std::string& Stage::Name() const { return name_; }

Task* Stage::FindTask(TaskConfig::TaskType task_type) const {
  auto iter = tasks_.find(task_type);
  if (iter == tasks_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

bool Stage::ExecuteTaskOnReferenceLine(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      return false;
    }

    auto ret = common::Status::OK();
    for (auto* task : task_list_) {
      ret = task->Execute(frame, &reference_line_info);
      if (!ret.ok()) {
        AERROR << "Failed to run tasks[" << task->Name()
               << "], Error message: " << ret.error_message();
        break;
      }
    }

    if (reference_line_info.speed_data().empty()) {
      *reference_line_info.mutable_speed_data() =
          SpeedProfileGenerator::GenerateFallbackSpeed();
      reference_line_info.AddCost(kSpeedOptimizationFallbackCost);
      reference_line_info.set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
    } else {
      reference_line_info.set_trajectory_type(ADCTrajectory::NORMAL);
    }
    DiscretizedTrajectory trajectory;
    if (!reference_line_info.CombinePathAndSpeedProfile(
            planning_start_point.relative_time(),
            planning_start_point.path_point().s(), &trajectory)) {
      AERROR << "Fail to aggregate planning trajectory.";
      return false;
    }
    reference_line_info.SetTrajectory(trajectory);
    reference_line_info.SetDrivable(true);
    return true;
  }
  return true;
}

bool Stage::ExecuteTaskOnOpenSpace(Frame* frame) {
  auto ret = common::Status::OK();
  for (auto* task : task_list_) {
    ret = task->Execute(frame);
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      return false;
    }
  }

  if (frame->open_space_info().fallback_flag()) {
    auto& trajectory = frame->open_space_info().fallback_trajectory().first;
    auto& gear = frame->open_space_info().fallback_trajectory().second;
    PublishableTrajectory publishable_trajectory(Clock::NowInSeconds(),
                                                 trajectory);
    auto publishable_traj_and_gear =
        std::make_pair(std::move(publishable_trajectory), gear);

    *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
        std::move(publishable_traj_and_gear);
  } else {
    auto& trajectory =
        frame->open_space_info().chosen_partitioned_trajectory().first;
    auto& gear =
        frame->open_space_info().chosen_partitioned_trajectory().second;
    PublishableTrajectory publishable_trajectory(Clock::NowInSeconds(),
                                                 trajectory);
    auto publishable_traj_and_gear =
        std::make_pair(std::move(publishable_trajectory), gear);

    *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
        std::move(publishable_traj_and_gear);
  }
  return true;
}

Stage::StageStatus Stage::FinishScenario() {
  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
