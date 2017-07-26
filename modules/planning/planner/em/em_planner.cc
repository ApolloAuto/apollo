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

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/data_center.h"
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

using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::TrajectoryPoint;
using apollo::common::vehicle_state::VehicleState;

EMPlanner::EMPlanner() {}

void EMPlanner::RegisterOptimizers() {
  optimizer_factory_.Register(DP_POLY_PATH_OPTIMIZER, []() -> Optimizer* {
    return new DpPolyPathOptimizer("DpPolyPathOptimizer");
  });
  optimizer_factory_.Register(DP_ST_SPEED_OPTIMIZER, []() -> Optimizer* {
    return new DpStSpeedOptimizer("DpStSpeedOptimizer");
  });
  optimizer_factory_.Register(QP_SPLINE_PATH_OPTIMIZER, []() -> Optimizer* {
    return new QPSplinePathOptimizer("QPSplinePathOptimizer");
  });
  optimizer_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Optimizer* {
    return new QpSplineStSpeedOptimizer("QpSplineStSpeedOptimizer");
  });
}

Status EMPlanner::Init(const PlanningConfig& config) {
  AINFO << "In EMPlanner::Init()";
  RegisterOptimizers();
  for (int i = 0; i < config.em_planner_config().optimizer_size(); ++i) {
    optimizers_.emplace_back(optimizer_factory_.CreateObject(
        config.em_planner_config().optimizer(i)));
    optimizers_.back()->Init();
  }
  routing_proxy_.Init();
  for (auto& optimizer : optimizers_) {
    if (!optimizer->Init()) {
      AERROR << common::util::StrCat("Init optimizer[", optimizer->name(),
                                     "] failed.");
      return Status(ErrorCode::PLANNING_ERROR, "Init optimizer failed.");
    }
  }
  return Status::OK();
}

Status EMPlanner::MakePlan(
    const TrajectoryPoint& start_point,
    std::vector<TrajectoryPoint>* discretized_trajectory) {
  DataCenter* data_center = DataCenter::instance();
  Frame* frame = data_center->current_frame();

  // FIXME(all): switch to real routing when it is ready
  GenerateReferenceLineFromRouting(routing_proxy_);

  frame->set_planning_data(new PlanningData());
  if (data_center->last_frame()) {
    ADEBUG << "last frame:" << data_center->last_frame()->DebugString();
  }
  ADEBUG << "start point:" << start_point.DebugString();
  auto planning_data = frame->mutable_planning_data();
  planning_data->set_init_planning_point(start_point);

  if (reference_line_) {
    ADEBUG << "reference line:" << reference_line_->DebugString();
  }
  planning_data->set_reference_line(reference_line_);
  std::shared_ptr<DecisionData> decision_data(new DecisionData());

  planning_data->set_decision_data(decision_data);
  for (auto& optimizer : optimizers_) {
    optimizer->Optimize(planning_data);
    ADEBUG << "after optimizer " << optimizer->name()
        << ":" << planning_data->DebugString();
  }
  PublishableTrajectory computed_trajectory;
  if (!planning_data->aggregate(FLAGS_output_trajectory_time_resolution,
                                &computed_trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  frame->set_computed_trajectory(computed_trajectory);
  *discretized_trajectory = computed_trajectory.trajectory_points();
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

Status EMPlanner::GenerateReferenceLineFromRouting(
    const RoutingProxy& routing_proxy) {
  DataCenter* data_center = DataCenter::instance();

  const auto& routing_result = routing_proxy.routing();
  const auto& map = data_center->map();
  std::vector<ReferencePoint> ref_points;
  common::math::Vec2d vehicle_position;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
  ReferenceLineSmoother smoother;
  smoother.SetConfig(smoother_config_);  // use the default value in config.

  vehicle_position.set_x(common::vehicle_state::VehicleState::instance()->x());
  vehicle_position.set_y(common::vehicle_state::VehicleState::instance()->y());

  for (const auto& lane : routing_result.route()) {
    hdmap::Id lane_id;
    lane_id.set_id(lane.id());
    ADEBUG << "Added lane from routing:" << lane.id();
    lane_info_ptr = map.get_lane_by_id(lane_id);
    if (!lane_info_ptr) {
      std::string msg("failed to find lane " + lane.id() + " from map ");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    const auto& points = lane_info_ptr->points();
    const auto& headings = lane_info_ptr->headings();
    for (size_t i = 0; i < points.size(); ++i) {
      ref_points.emplace_back(points[i], headings[i], 0.0, 0.0, -2.0, 2.0);
    }
  }
  if (ref_points.empty()) {
    std::string msg("Found no reference points from map");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::unique_ptr<ReferenceLine> reference_line(new ReferenceLine(ref_points));
  std::vector<ReferencePoint> smoothed_ref_points;
  if (!smoother.smooth(*reference_line, vehicle_position,
                       &smoothed_ref_points)) {
    std::string msg("Fail to smooth a reference line from map");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ADEBUG << "smooth reference points num:" << smoothed_ref_points.size();
  reference_line_.reset(new ReferenceLine(smoothed_ref_points));
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
