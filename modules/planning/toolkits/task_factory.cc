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

#include "modules/planning/toolkits/task_factory.h"

#include <memory>
#include <string>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_piecewise_jerk_path/qp_piecewise_jerk_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"
#include "modules/planning/toolkits/task.h"

namespace apollo {
namespace planning {

apollo::common::util::Factory<TaskConfig::TaskType, Task,
                              Task* (*)(const TaskConfig& config)>
    TaskFactory::task_factory_;

void TaskFactory::Init(const PlanningConfig& config) {
  task_factory_.Register(TaskConfig::DP_ST_SPEED_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new DpStSpeedOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::QP_SPLINE_PATH_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new QpSplinePathOptimizer(config);
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
  task_factory_.Register(TaskConfig::POLY_ST_SPEED_OPTIMIZER,
                         [](const TaskConfig& config) -> Task* {
                           return new PolyStSpeedOptimizer(config);
                         });
  task_factory_.Register(TaskConfig::DECIDER_CREEP,
                         [](const TaskConfig& config) -> Task* {
                           return new DeciderCreep(config);
                         });
}

std::unique_ptr<Task> TaskFactory::CreateTask(const TaskConfig& task_config) {
  return task_factory_.CreateObject(task_config.task_type(), task_config);
}

}  // namespace planning
}  // namespace apollo
