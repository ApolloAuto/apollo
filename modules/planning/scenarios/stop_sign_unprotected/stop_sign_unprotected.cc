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

#include <limits>

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected.h"  // NOINT

#include "modules/planning/proto/planning_config.pb.h"

#include "cybertron/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::ErrorCode;
using common::Status;
using common::TrajectoryPoint;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::hdmap::StopSignInfo;
using apollo::hdmap::StopSignInfoConstPtr;


int StopSignUnprotectedScenario::current_stage_index_ = 0;

void StopSignUnprotectedScenario::RegisterTasks() {
  // deciders
  task_factory_.Register(DECIDER_CREEP,
                         []() -> Task* { return new DeciderCreep(); });
  // optimizers
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

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_stop_sign_unprotected_config_file, &config_));

  RegisterTasks();

  is_init_ = true;
  status_ = STATUS_INITED;

  return true;
}

void StopSignUnprotectedScenario::Observe(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindNextStopSign(reference_line_info)) {
    ADEBUG << "no stop sign found";
    return;
  }

  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  adc_distance_to_stop_sign_ =
      next_stop_sign_overlap_.start_s - adc_front_edge_s;
}

Status StopSignUnprotectedScenario::Process(
    const TrajectoryPoint& planning_start_point,
    Frame* frame) {
  status_ = STATUS_PROCESSING;

  if (!InitTasks(config_, current_stage_index_, &tasks_)) {
    return Status(ErrorCode::PLANNING_ERROR, "failed to init tasks");
  }

  Status status = Status(ErrorCode::PLANNING_ERROR,
                         "Failed to process stage in stop_sign_upprotected.");
  switch (stage_) {
    case StopSignUnprotectedStage::PRE_STOP: {
      status = PreStop(frame);
      break;
    }
    case StopSignUnprotectedStage::STOP: {
      status = Stop(frame);
      break;
    }
    case StopSignUnprotectedStage::CREEP: {
      status = Creep(planning_start_point, frame);
      break;
    }
    case StopSignUnprotectedStage::INTERSECTION_CRUISE: {
      status = IntersectionCruise(planning_start_point, frame);
      break;
    }
    default:
      break;
  }

  return Status::OK();
}

bool StopSignUnprotectedScenario::IsTransferable(
    const Scenario& current_scenario,
    const common::TrajectoryPoint& ego_point,
    const Frame& frame) const {
  // TODO(all): move to scenario conf later
  constexpr uint32_t conf_start_stop_sign_timer = 10;  // second

  if (next_stop_sign_ == nullptr) {
    return false;
  }

  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  const uint32_t time_distance = ceil(adc_distance_to_stop_sign_ / adc_speed);

  switch (current_scenario.scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
    case ScenarioConfig::APPROACH:
      return time_distance <= conf_start_stop_sign_timer ? true : false;
    case ScenarioConfig::STOP_SIGN_PROTECTED:
      return false;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      return true;
    case ScenarioConfig::TRAFFIC_LIGHT_LEFT_TURN_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_LEFT_TURN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_GO_THROUGH:
    default:
      break;
  }

  return false;
}

common::Status StopSignUnprotectedScenario::PreStop(
    Frame* frame) {
  // TODO(all)
  return Status::OK();
}

common::Status StopSignUnprotectedScenario::Stop(
    Frame* frame) {
  // TODO(all)
  return Status::OK();
}

common::Status StopSignUnprotectedScenario::Creep(
    const common::TrajectoryPoint& planning_start_point,
    Frame* frame) {
  // TODO(all)
  return Status::OK();
}

common::Status StopSignUnprotectedScenario::IntersectionCruise(
    const common::TrajectoryPoint& planning_start_point,
    Frame* frame) {
  // TODO(all)
  return Status::OK();
}

/**
 * @brief: fine next stop sign ahead of adc along reference line
 */
bool StopSignUnprotectedScenario::FindNextStopSign(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  // TODO(all): move to scenario conf later
  constexpr double conf_min_pass_s_distance = 3.0;

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  double min_start_s = std::numeric_limits<double>::max();
  for (const PathOverlap& stop_sign_overlap : stop_sign_overlaps) {
    if (adc_front_edge_s - stop_sign_overlap.end_s <=
        conf_min_pass_s_distance &&
        stop_sign_overlap.start_s < min_start_s) {
      min_start_s = stop_sign_overlap.start_s;
      next_stop_sign_overlap_ = stop_sign_overlap;
    }
  }

  if (next_stop_sign_overlap_.object_id.empty()) {
    return false;
  }

  next_stop_sign_ = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(next_stop_sign_overlap_.object_id));
  if (!next_stop_sign_) {
    AERROR << "Could not find stop sign: " << next_stop_sign_overlap_.object_id;
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
