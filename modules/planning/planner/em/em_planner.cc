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
#include <memory>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/em_planning_data.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/optimizer/dp_poly_path/dp_poly_path_optimizer.h"
// #include "modules/planning/optimizer/dp_st_speed/dp_st_speed_optimizer.h"
// #include "modules/planning/optimizer/qp_spline_path/qp_spline_path_optimizer.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::TrajectoryPoint;
using apollo::common::vehicle_state::VehicleState;

EMPlanner::EMPlanner() {

}

void EMPlanner::RegisterOptimizers() {
  optimizer_factory_.Register(
      DP_POLY_PATH_OPTIMIZER,
      []() -> Optimizer* { return new DpPolyPathOptimizer("DpPolyPathOptimizer"); });
  // optimizer_factory_.Register(
  //     DP_ST_SPEED_OPTIMIZER,
  //     []() -> Optimizer* { return new DpStSpeedOptimizer("DpStSpeedOptimizer"); });
  // optimizer_factory_.Register(
  //     QP_SPLINE_PATH_OPTIMIZER,
  //     []() -> Optimizer* { return new QPSplinePathOptimizer("QPSplinePathOptimizer"); });
}

Status EMPlanner::Init(const PlanningConfig& config) {
  RegisterOptimizers();
  for (int i = 0; i < config.em_planner_config().optimizer_size(); ++i) {
    optimizers_.emplace_back(optimizer_factory_.CreateObject(
        config.em_planner_config().optimizer(i)));
  }
  return Status::OK();
}

Status EMPlanner::MakePlan(const TrajectoryPoint& start_point,
                         std::vector<TrajectoryPoint>* discretized_trajectory) {
  DataCenter* data_center = DataCenter::instance();
  Frame* frame = data_center->current_frame();

  frame->set_planning_data(new EMPlanningData());
  if (data_center->last_frame()) {
    ADEBUG << "last frame:" << data_center->last_frame()->DebugString();
  }
  ADEBUG << "start point:" << start_point.DebugString();
  frame->mutable_planning_data()->set_init_planning_point(start_point);

  //  frame->mutable_planning_data()->set_reference_line(reference_line);
  //  frame->mutable_planning_data()->set_decision_data(decision_data);

  return Status::OK();
}

std::vector<SpeedPoint> EMPlanner::GenerateInitSpeedProfile(
    const double init_v, const double init_a) {
  // TODO(@lianglia_apollo): this is a dummy simple hot start, need refine later
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

}  // namespace planning
}  // namespace apollo
