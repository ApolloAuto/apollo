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

#include "modules/planning/planning_base/scenario_base/stage.h"

#include <unordered_map>
#include <utility>

#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/time/clock.h"
#include "modules/planning/planning_base/common/config_util.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planning_base/task_base/task.h"

namespace apollo {
namespace planning {

using apollo::cyber::Clock;

namespace {
constexpr double kSpeedOptimizationFallbackCost = 2e4;
}  // namespace

Stage::Stage()
    : next_stage_(""), context_(nullptr), injector_(nullptr), name_("") {}

bool Stage::Init(const StagePipeline& config,
                 const std::shared_ptr<DependencyInjector>& injector,
                 const std::string& config_dir, void* context) {
  pipeline_config_ = config;
  next_stage_ = config.name();
  injector_ = injector;
  name_ = config.name();
  context_ = context;
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(name_);
  std::string path_name = ConfigUtil::TransformToPathName(name_);
  // Load task plugin
  for (int i = 0; i < pipeline_config_.task_size(); ++i) {
    auto task = pipeline_config_.task(i);
    auto task_type = task.type();
    auto task_ptr = apollo::cyber::plugin_manager::PluginManager::Instance()
                        ->CreateInstance<Task>(
                            ConfigUtil::GetFullPlanningClassName(task_type));
    if (nullptr == task_ptr) {
      AERROR << "Create task " << task.name() << " of " << name_ << " failed!";
      return false;
    }
    std::string task_config_dir = config_dir + "/" + path_name;
    task_ptr->Init(task_config_dir, task.name(), injector);
    task_list_.push_back(task_ptr);
    tasks_[task.name()] = task_ptr;
  }
  return true;
}

const std::string& Stage::Name() const { return name_; }

Task* Stage::FindTask(const std::string& task_type) const {
  auto iter = tasks_.find(task_type);
  if (iter == tasks_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

StageResult Stage::ExecuteTaskOnReferenceLine(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  StageResult stage_result;
  if (frame->reference_line_info().empty()) {
    AERROR << "referenceline is empty in stage" << name_;
    return stage_result.SetStageStatus(StageStatusType::ERROR);
  }
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      return stage_result.SetStageStatus(StageStatusType::ERROR);
    }

    for (auto task : task_list_) {
      const double start_timestamp = Clock::NowInSeconds();

      const auto ret = task->Execute(frame, &reference_line_info);

      const double end_timestamp = Clock::NowInSeconds();
      const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
      ADEBUG << "after task[" << task->Name()
             << "]: " << reference_line_info.PathSpeedDebugString();
      ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
      RecordDebugInfo(&reference_line_info, task->Name(), time_diff_ms);

      if (!ret.ok()) {
        stage_result.SetTaskStatus(ret);
        AERROR << "Failed to run tasks[" << task->Name()
               << "], Error message: " << ret.error_message();
        break;
      }
    }

    if (reference_line_info.speed_data().empty()) {
      *reference_line_info.mutable_speed_data() =
          SpeedProfileGenerator::GenerateFallbackSpeed(
              injector_->ego_info(), FLAGS_speed_fallback_distance);
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
      return stage_result.SetStageStatus(StageStatusType::ERROR);
    }
    reference_line_info.SetTrajectory(trajectory);
    reference_line_info.SetDrivable(true);
    return stage_result;
  }
  return stage_result;
}

StageResult Stage::ExecuteTaskOnReferenceLineForOnlineLearning(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  // online learning mode
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    reference_line_info.SetDrivable(false);
  }

  StageResult stage_result;
  // FIXME(all): current only pick up the first reference line to use
  // learning model trajectory
  auto& picked_reference_line_info =
      frame->mutable_reference_line_info()->front();
  for (auto task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();

    const auto ret = task->Execute(frame, &picked_reference_line_info);

    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    ADEBUG << "task[" << task->Name() << "] time spent: " << time_diff_ms
           << " ms.";
    RecordDebugInfo(&picked_reference_line_info, task->Name(), time_diff_ms);

    if (!ret.ok()) {
      stage_result.SetTaskStatus(ret);
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

  return stage_result;
}

StageResult Stage::ExecuteTaskOnOpenSpace(Frame* frame) {
  auto ret = common::Status::OK();
  StageResult stage_result;
  for (auto task : task_list_) {
    ret = task->Execute(frame);
    if (!ret.ok()) {
      stage_result.SetTaskStatus(ret);
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      return stage_result;
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
  return stage_result;
}

StageResult Stage::FinishScenario() {
  next_stage_ = "";
  return StageResult(StageStatusType::FINISHED);
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

}  // namespace planning
}  // namespace apollo
