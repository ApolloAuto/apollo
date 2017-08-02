/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planner/em/em_planner.h"

#include <fstream>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_data.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/optimizer/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/optimizer/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"

namespace apollo {
namespace planning {

using common::Status;
using common::adapter::AdapterManager;
using common::time::Clock;
using common::ErrorCode;
using common::SpeedPoint;
using common::TrajectoryPoint;
using common::VehicleState;

EMPlanner::EMPlanner() {}

void EMPlanner::RegisterOptimizers() {
  optimizer_factory_.Register(DP_POLY_PATH_OPTIMIZER, []() -> Optimizer* {
    return new DpPolyPathOptimizer(OptimizerType_Name(DP_POLY_PATH_OPTIMIZER));
  });
  optimizer_factory_.Register(DP_ST_SPEED_OPTIMIZER, []() -> Optimizer* {
    return new DpStSpeedOptimizer(OptimizerType_Name(DP_ST_SPEED_OPTIMIZER));
  });
  optimizer_factory_.Register(QP_SPLINE_PATH_OPTIMIZER, []() -> Optimizer* {
    return new QpSplinePathOptimizer(
        OptimizerType_Name(QP_SPLINE_PATH_OPTIMIZER));
  });
  optimizer_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Optimizer* {
    return new QpSplineStSpeedOptimizer(
        OptimizerType_Name(QP_SPLINE_ST_SPEED_OPTIMIZER));
  });
}

Status EMPlanner::Init(const PlanningConfig& config) {
  AINFO << "In EMPlanner::Init()";
  RegisterOptimizers();
  for (int i = 0; i < config.em_planner_config().optimizer_size(); ++i) {
    optimizers_.emplace_back(optimizer_factory_.CreateObject(
        config.em_planner_config().optimizer(i)));
    AINFO << "Created optimizer:" << optimizers_.back()->name();
  }
  for (auto& optimizer : optimizers_) {
    if (!optimizer->Init()) {
      std::string msg(common::util::StrCat("Init optimizer[", optimizer->name(),
                                           "] failed."));
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

void EMPlanner::RecordDebugInfo(const std::string& name,
                                PlanningData* planning_data,
                                double time_diff_ms) {
  OptimizerType type;
  DCHECK(OptimizerType_Parse(name, &type));

  if (type == DP_POLY_PATH_OPTIMIZER || type == QP_SPLINE_PATH_OPTIMIZER) {
    em_planner_debugger_.paths_[name].first =
        planning_data->path_data().discretized_path();
  } else if (type == DP_ST_SPEED_OPTIMIZER ||
             type == QP_SPLINE_ST_SPEED_OPTIMIZER) {
    em_planner_debugger_.speed_profiles_[name].first =
        planning_data->speed_data().speed_vector();
  }
  em_planner_debugger_.speed_profiles_[name].second = time_diff_ms;
}

Status EMPlanner::Plan(const TrajectoryPoint& planning_start_point,
                       Frame* frame,
                       PublishableTrajectory* ptr_publishable_trajectory) {
  if (!frame) {
    AERROR << "Frame is empty in EMPlanner";
    return Status(ErrorCode::PLANNING_ERROR, "Frame is null");
  }

  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  auto* planning_data = frame->mutable_planning_data();

  std::shared_ptr<DecisionData> decision_data(new DecisionData());

  planning_data->set_decision_data(decision_data);
  for (auto& optimizer : optimizers_) {
    const double start_timestamp = apollo::common::time::ToSecond(Clock::Now());
    optimizer->Optimize(frame);
    const double end_timestamp = apollo::common::time::ToSecond(Clock::Now());
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "after optimizer " << optimizer->name() << ":"
           << planning_data->DebugString();

    if (FLAGS_enable_record_debug) {
      RecordDebugInfo(optimizer->name(), planning_data, time_diff_ms);
    }
  }
  PublishableTrajectory computed_trajectory;
  if (!planning_data->aggregate(FLAGS_output_trajectory_time_resolution,
                                planning_start_point.relative_time(),
                                &computed_trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  computed_trajectory.set_header_time(VehicleState::instance()->timestamp());
  frame->set_computed_trajectory(computed_trajectory);
  //  computed_trajectory.populate_trajectory_protobuf(trajectory_pb);
  *ptr_publishable_trajectory = std::move(computed_trajectory);

  // Add debug information.
  if (FLAGS_enable_record_debug) {
    em_planner_debugger_.reference_line_ =
        planning_data->reference_line().reference_points();
  }
  return Status::OK();
}

std::vector<SpeedPoint> EMPlanner::GenerateInitSpeedProfile(
    const double init_v, const double init_a) {
  // TODO(@lianglia-apollo): this is a dummy simple hot start, need refine later
  std::array<double, 3> start_state;

  // distance 0.0
  start_state[0] = 0.0;

  // start velocity
  start_state[1] = init_v;

  // start acceleration
  start_state[2] = init_a;

  std::array<double, 2> end_state;
  // end state velocity
  end_state[0] = 10.0;

  // end state acceleration
  end_state[1] = 0.0;

  // pre assume the curve time is 8 second, can be change later
  QuarticPolynomialCurve1d speed_curve(start_state, end_state,
                                       FLAGS_trajectory_time_length);

  // assume the time resolution is 0.1
  std::uint32_t num_time_steps =
      static_cast<std::uint32_t>(FLAGS_trajectory_time_length /
                                 FLAGS_trajectory_time_resolution) +
      1;
  std::vector<SpeedPoint> speed_profile;
  speed_profile.reserve(num_time_steps);

  for (std::uint32_t i = 0; i < num_time_steps; ++i) {
    double t = i * FLAGS_trajectory_time_resolution;
    double s = speed_curve.evaluate(0, t);
    double v = speed_curve.evaluate(1, t);
    double a = speed_curve.evaluate(2, t);
    double da = speed_curve.evaluate(3, t);
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_point.set_da(da);
    speed_profile.push_back(speed_point);
  }
  return std::move(speed_profile);
}

EMPlannerDebugger& EMPlanner::em_planner_debugger() {
  return em_planner_debugger_;
}

}  // namespace planning
}  // namespace apollo
