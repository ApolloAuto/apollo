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

#include "cyber/time/clock.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::cyber::Clock;

namespace {
// constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
// constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

Stage::Stage(const ScenarioConfig::StageConfig& config,
             const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {
  // set stage_type in PlanningContext
  injector->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(stage_type());

  name_ = StageType_Name(config_.stage_type());
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
      auto ptr = TaskFactory::CreateTask(*config_map[task_type], injector_);
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

    for (auto* task : task_list_) {
      const double start_timestamp = Clock::NowInSeconds();

      const auto ret = task->Execute(frame, &reference_line_info);

      const double end_timestamp = Clock::NowInSeconds();
      const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
      ADEBUG << "after task[" << task->Name()
             << "]: " << reference_line_info.PathSpeedDebugString();
      ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
      RecordDebugInfo(&reference_line_info, task->Name(), time_diff_ms);

      if (!ret.ok()) {
        AERROR << "Failed to run tasks[" << task->Name()
               << "], Error message: " << ret.error_message();
        break;
      }
    }

    if (reference_line_info.speed_data().empty()) {
      *reference_line_info.mutable_speed_data() =
          SpeedProfileGenerator::GenerateFallbackSpeed(injector_->ego_info());
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

bool Stage::ExecuteTaskOnReferenceLineForOnlineLearning(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  // online learning mode
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    reference_line_info.SetDrivable(false);
  }

  // FIXME(all): current only pick up the first reference line to use
  // learning model trajectory
  auto& picked_reference_line_info =
      frame->mutable_reference_line_info()->front();
  for (auto* task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    const auto ret = task->Execute(frame, &picked_reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "task[" << task->Name() << "] time spent: " << time_diff_ms
           << " ms.";
    RecordDebugInfo(&picked_reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
  }

  const std::vector<common::TrajectoryPoint>& adc_future_trajectory_points =
      picked_reference_line_info.trajectory();
  DiscretizedTrajectory trajectory;
  if (picked_reference_line_info.AdjustTrajectoryWhichStartsFromCurrentPos(
          planning_start_point, adc_future_trajectory_points, &trajectory)) {
    picked_reference_line_info.SetTrajectory(trajectory);
    picked_reference_line_info.SetDrivable(true);
    picked_reference_line_info.SetCost(0);
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
    PublishableTrajectory publishable_trajectory(
        Clock::NowInSeconds(), trajectory);
    auto publishable_traj_and_gear =
        std::make_pair(std::move(publishable_trajectory), gear);

    *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
        std::move(publishable_traj_and_gear);
  } else {
    auto& trajectory =
        frame->open_space_info().chosen_partitioned_trajectory().first;
    auto& gear =
        frame->open_space_info().chosen_partitioned_trajectory().second;
    PublishableTrajectory publishable_trajectory(
        Clock::NowInSeconds(), trajectory);
    auto publishable_traj_and_gear =
        std::make_pair(std::move(publishable_trajectory), gear);

    *(frame->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
        std::move(publishable_traj_and_gear);
  }
  return true;
}

Stage::StageStatus Stage::FinishScenario() {
  next_stage_ = StageType::NO_STAGE;
  return Stage::FINISHED;
}

void Stage::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                            const std::string& name,
                            const double time_diff_ms) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }

  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
