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

#include "modules/planning/tasks/task_factory.h"

#include "modules/common/status/status.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/creep_decider/creep_decider.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "modules/planning/tasks/deciders/open_space_decider/open_space_fallback_decider.h"
#include "modules/planning/tasks/deciders/open_space_decider/open_space_pre_stop_decider.h"
#include "modules/planning/tasks/deciders/open_space_decider/open_space_roi_decider.h"
#include "modules/planning/tasks/deciders/path_assessment_decider/path_assessment_decider.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"
#include "modules/planning/tasks/deciders/path_decider/path_decider.h"
#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"
#include "modules/planning/tasks/deciders/path_reference_decider/path_reference_decider.h"
#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"
#include "modules/planning/tasks/deciders/rss_decider/rss_decider.h"
#include "modules/planning/tasks/deciders/rule_based_stop_decider/rule_based_stop_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"
#include "modules/planning/tasks/learning_model/learning_model_inference_task.h"
#include "modules/planning/tasks/learning_model/learning_model_inference_trajectory_task.h"
#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"
#include "modules/planning/tasks/optimizers/open_space_trajectory_partition/open_space_trajectory_partition.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"
#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"
#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_nonlinear_optimizer.h"
#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

apollo::common::util::Factory<
    TaskConfig::TaskType, Task,
    Task* (*)(const TaskConfig& config,
              const std::shared_ptr<DependencyInjector>& injector),
    std::unordered_map<
        TaskConfig::TaskType,
        Task* (*)(const TaskConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector),
        std::hash<int>>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

void TaskFactory::Init(const PlanningConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector) {
  ///////////////////////////
  // deciders
  task_factory_.Register(
      TaskConfig::CREEP_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new CreepDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LANE_CHANGE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LaneChangeDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_FALLBACK_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceFallbackDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_PRE_STOP_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpacePreStopDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_ROI_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceRoiDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_ASSESSMENT_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathAssessmentDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_BOUNDS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_LANE_BORROW_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathLaneBorrowDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_REFERENCE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathReferenceDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_REUSE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathReuseDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::RSS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new RssDecider(config);
      });
  task_factory_.Register(
      TaskConfig::RULE_BASED_STOP_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new RuleBasedStopDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SPEED_BOUNDS_FINAL_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SPEED_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::ST_BOUNDS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new STBoundsDecider(config, injector);
      });
  ///////////////////////////
  // optimizers
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_TRAJECTORY_PARTITION,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceTrajectoryPartition(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_TRAJECTORY_PROVIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceTrajectoryProvider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PiecewiseJerkSpeedNonlinearOptimizer(config);
      });
  task_factory_.Register(
      TaskConfig::PIECEWISE_JERK_PATH_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PiecewiseJerkPathOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PIECEWISE_JERK_SPEED_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PiecewiseJerkSpeedOptimizer(config);
      });
  task_factory_.Register(
      TaskConfig::SPEED_HEURISTIC_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathTimeHeuristicOptimizer(config);
      });
  ///////////////////////////
  // other tasks
  task_factory_.Register(
      TaskConfig::LEARNING_MODEL_INFERENCE_TASK,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LearningModelInferenceTask(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LEARNING_MODEL_INFERENCE_TRAJECTORY_TASK,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LearningModelInferenceTrajectoryTask(config, injector);
      });

  for (const auto& default_task_config : config.default_task_config()) {
    default_task_configs_[default_task_config.task_type()] =
        default_task_config;
  }
}

std::unique_ptr<Task> TaskFactory::CreateTask(
    const TaskConfig& task_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  TaskConfig merged_config;
  if (default_task_configs_.find(task_config.task_type()) !=
      default_task_configs_.end()) {
    merged_config = default_task_configs_[task_config.task_type()];
  }
  merged_config.MergeFrom(task_config);
  return task_factory_.CreateObject(task_config.task_type(), merged_config,
                                    injector);
}

}  // namespace planning
}  // namespace apollo
