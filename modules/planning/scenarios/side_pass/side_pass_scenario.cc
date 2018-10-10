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

#include "modules/planning/scenarios/side_pass/side_pass_scenario.h"

#include <fstream>
#include <limits>
#include <utility>

#include "cybertron/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::ErrorCode;
using common::SLPoint;
using common::SpeedPoint;
using common::Status;
using common::TrajectoryPoint;
using common::math::Vec2d;
using common::time::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

void SidePassScenario::RegisterTasks() {
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

bool SidePassScenario::Init() {
  if (is_init_) {
    return true;
  }

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_side_pass_config_file, &config_));

  RegisterTasks();

  is_init_ = true;
  status_ = STATUS_INITED;

  return true;
}

Status SidePassScenario::Process(const TrajectoryPoint& planning_start_point,
                                 Frame* frame) {
  status_ = STATUS_PROCESSING;

  // init tasks
  std::string stage_name = "";
  if (stage_ == SidePassStage::OBSTACLE_APPROACH) {
    stage_name = FLAGS_scenario_side_pass_stage_obstacle_approach;
  } else if (stage_ == SidePassStage::PATH_GENERATION) {
    stage_name = FLAGS_scenario_side_pass_stage_path_generation;
  } else if (stage_ == SidePassStage::WAITPOINT_STOP) {
    stage_name = FLAGS_scenario_side_pass_stage_waitpoint_stop;
  } else if (stage_ == SidePassStage::SAFETY_DETECTION) {
    stage_name = FLAGS_scenario_side_pass_stage_safety_detection;
  } else if (stage_ == SidePassStage::OBSTACLE_PASS) {
    stage_name = FLAGS_scenario_side_pass_stage_obstacle_pass;
  } else if (stage_ == SidePassStage::DONE) {
    status_ = STATUS_DONE;
    return Status(ErrorCode::OK, "side_pass DONE");
  }
  if (!InitTasks(config_, stage_name, &tasks_)) {
    return Status(ErrorCode::PLANNING_ERROR, "failed to init tasks");
  }

  // TODO(all)

  Status status = Status(ErrorCode::PLANNING_ERROR,
                         "Failed to process stage in side pass.");
  switch (stage_) {
    case SidePassStage::OBSTACLE_APPROACH: {
      status = ApproachObstacle(planning_start_point, frame);
      break;
    }
    case SidePassStage::PATH_GENERATION: {
      status = GeneratePath(planning_start_point, frame);
      break;
    }
    case SidePassStage::WAITPOINT_STOP: {
      status = StopOnWaitPoint(planning_start_point, frame);
      break;
    }
    case SidePassStage::SAFETY_DETECTION: {
      status = DetectSafety(planning_start_point, frame);
      break;
    }
    case SidePassStage::OBSTACLE_PASS: {
      status = PassObstacle(planning_start_point, frame);
      break;
    }
    default:
      break;
  }

  return status;
}

bool SidePassScenario::IsTransferable(const Scenario& current_scenario,
                                      const common::TrajectoryPoint& ego_point,
                                      const Frame& frame) const {
  // TODO(All): implement here
  return false;
}

Status SidePassScenario::ApproachObstacle(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  return Status::OK();
}

Status SidePassScenario::GeneratePath(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  return Status::OK();
}

Status SidePassScenario::StopOnWaitPoint(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  return Status::OK();
}

Status SidePassScenario::DetectSafety(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  return Status::OK();
}

Status SidePassScenario::PassObstacle(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
