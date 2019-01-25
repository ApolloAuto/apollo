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

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/tasks/deciders/decider_creep.h"
#include "modules/planning/tasks/deciders/decider_rule_based_stop.h"
#include "modules/planning/tasks/deciders/side_pass_path_decider.h"
#include "modules/planning/tasks/deciders/side_pass_safety.h"
#include "modules/planning/tasks/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/tasks/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/tasks/optimizers/path_decider/path_decider.h"
#include "modules/planning/tasks/optimizers/proceed_with_caution_speed/proceed_with_caution_speed_generator.h"
#include "modules/planning/tasks/optimizers/qp_piecewise_jerk_path/qp_piecewise_jerk_path_optimizer.h"
#include "modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/tasks/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/tasks/optimizers/speed_decider/speed_decider.h"
#include "modules/planning/tasks/rss/decider_rss.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

apollo::common::util::Factory<
    TaskConfig::TaskType, Task, Task* (*)(const TaskConfig& config),
    std::unordered_map<TaskConfig::TaskType,
                       Task* (*)(const TaskConfig& config), std::hash<int>>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

void TaskFactory::Init(const PlanningConfig& config) {
  task_factory_.Register(TaskConfig::DP_ST_SPEED_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new DpStSpeedOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::QP_SPLINE_PATH_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new QpSplinePathOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::SIDE_PASS_PATH_DECIDER,
                         [](const TaskConfig& config) -> Task* {
                           return new SidePassPathDecider(config);
                         });
  task_factory_.Register(TaskConfig::DP_POLY_PATH_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new DpPolyPathOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::PATH_DECIDER,
                         [](const TaskConfig& config) -> Task* {
                           return new PathDecider(config);
                         });
  task_factory_.Register(TaskConfig::SPEED_DECIDER,
                         [](const TaskConfig& config) -> Task* {
                           return new SpeedDecider(config);
                         });
  task_factory_.Register(TaskConfig::QP_SPLINE_ST_SPEED_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new QpSplineStSpeedOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::QP_PIECEWISE_JERK_PATH_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new QpPiecewiseJerkPathOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::PROCEED_WITH_CAUTION_SPEED,
                         [](const TaskConfig& config) -> Task* {
                           return new ProceedWithCautionSpeedGenerator(config);
                         });
  task_factory_.Register(TaskConfig::DECIDER_CREEP,
                         [](const TaskConfig& config) -> Task* {
                           return new DeciderCreep(config);
                         });
  task_factory_.Register(TaskConfig::DECIDER_RULE_BASED_STOP,
                         [](const TaskConfig& config) -> Task* {
                           return new DeciderRuleBasedStop(config);
                         });
  task_factory_.Register(TaskConfig::SIDE_PASS_SAFETY,
                         [](const TaskConfig& config) -> Task* {
                           return new SidePassSafety(config);
                         });
  task_factory_.Register(TaskConfig::DECIDER_RSS,
                         [](const TaskConfig& config) -> Task* {
                           return new RssDecider(config);
                         });
  for (const auto& default_task_config : config.default_task_config()) {
    default_task_configs_[default_task_config.task_type()] =
        default_task_config;
  }
}

std::unique_ptr<Task> TaskFactory::CreateTask(const TaskConfig& task_config) {
  TaskConfig merged_config;
  if (default_task_configs_.find(task_config.task_type()) !=
      default_task_configs_.end()) {
    merged_config = default_task_configs_[task_config.task_type()];
  }
  merged_config.MergeFrom(task_config);
  return task_factory_.CreateObject(task_config.task_type(), merged_config);
}

}  // namespace planning
}  // namespace apollo
