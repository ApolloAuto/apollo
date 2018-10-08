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

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected.h"  // NOINT


#include "cybertron/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::Status;
using common::TrajectoryPoint;

void StopSignUnprotectedScenario::RegisterTasks() {
  task_factory_.Register(DP_POLY_PATH_OPTIMIZER,
                         []() -> Task* { return new DpPolyPathOptimizer(); });
  task_factory_.Register(PATH_DECIDER,
                         []() -> Task* { return new PathDecider(); });
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new DpStSpeedOptimizer(); });
  task_factory_.Register(SPEED_DECIDER,
                         []() -> Task* { return new SpeedDecider(); });
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Task* {
    return new QpSplineStSpeedOptimizer();
  });
}

bool StopSignUnprotectedScenario::Init() {
  if (is_init_) {
    return true;
  }
  RegisterTasks();

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_stop_sign_unprotected_config_file, &config_));

  // get all active tasks
  std::vector<TaskType> tasks;
  CHECK_GT(config_.stage_size(), 0);
  // TODO(All): deal with multiple stages.
  for (int i = 0; i < config_.stage(0).task_size(); ++i) {
    tasks.push_back(config_.stage(0).task(i));
  }

  // init task with conf
  for (int i = 0; i < config_.scenario_task_config_size(); ++i) {
    auto task_cfg = config_.scenario_task_config(i);
    TaskType task_type = task_cfg.task_type();
    if (std::find(tasks.begin(), tasks.end(), task_type) != tasks.end()) {
      tasks_.emplace_back(task_factory_.CreateObject(task_type));
      AINFO << "Created task:" << TaskType_Name(task_type);
      if (!tasks_.back()->Init(task_cfg)) {
        AERROR << "Init task[" << TaskType_Name(task_type) << "] failed.";
        return false;
      }
    }
  }

  is_init_ = true;
  return true;
}

Status StopSignUnprotectedScenario::Process(
    const TrajectoryPoint& planning_start_point,
    Frame* frame) {
  return Status::OK();
}

bool StopSignUnprotectedScenario::IsTransferable(
    const Scenario& current_scenario,
    const common::TrajectoryPoint& ego_point,
    const Frame& frame) const {
  // TODO(All): implement here
  return false;
}

}  // namespace planning
}  // namespace apollo
