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
#include "modules/planning/planner/em/decider.h"

namespace apollo {
namespace planning {

using common::Status;
using common::adapter::AdapterManager;
using common::time::Clock;
using common::ErrorCode;
using common::SpeedPoint;
using common::SLPoint;
using common::TrajectoryPoint;
using common::VehicleState;
using common::math::Vec2d;

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
                                const PlanningData* planning_data,
                                const double time_diff_ms,
                                planning_internal::Debug* ptr_debug,
                                planning::LatencyStats* ptr_latency_stats) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  OptimizerType type;
  DCHECK(OptimizerType_Parse(name, &type));
  if (type == DP_POLY_PATH_OPTIMIZER || type == QP_SPLINE_PATH_OPTIMIZER) {
    const auto& path_points =
        planning_data->path_data().discretized_path().path_points();
    auto ptr_optimized_path = ptr_debug->mutable_planning_data()->add_path();
    ptr_optimized_path->set_name(name);
    ptr_optimized_path->mutable_path_point()->CopyFrom(
        {path_points.begin(), path_points.end()});
  } else if (type == DP_ST_SPEED_OPTIMIZER ||
             type == QP_SPLINE_ST_SPEED_OPTIMIZER) {
    const auto& speed_points = planning_data->speed_data().speed_vector();
    auto ptr_speed_plan = ptr_debug->mutable_planning_data()->add_speed_plan();
    ptr_speed_plan->set_name(name);
    ptr_speed_plan->mutable_speed_point()->CopyFrom(
        {speed_points.begin(), speed_points.end()});
  }

  auto ptr_stats = ptr_latency_stats->add_processor_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

void EMPlanner::PopulateDecision(Frame* frame) {
  auto planning_pb = frame->MutableADCTrajectory();
  Decider decider(planning_pb->mutable_decision());
  decider.MakeDecision(frame);
}

Status EMPlanner::Plan(const TrajectoryPoint& planning_start_point,
                       Frame* frame,
                       DiscretizedTrajectory* ptr_trajectory) {
  if (!frame) {
    AERROR << "Frame is empty in EMPlanner";
    return Status(ErrorCode::PLANNING_ERROR, "Frame is null");
  }

  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  auto* planning_data = frame->mutable_planning_data();
  auto* heuristic_speed_data = planning_data->mutable_speed_data();
  heuristic_speed_data->set_speed_vector(
      GenerateInitSpeedProfile(planning_start_point));

  auto ptr_debug = frame->MutableADCTrajectory()->mutable_debug();
  auto ptr_latency_stats =
      frame->MutableADCTrajectory()->mutable_latency_stats();
  for (auto& optimizer : optimizers_) {
    const double start_timestamp = apollo::common::time::ToSecond(Clock::Now());
    optimizer->Optimize(frame);
    const double end_timestamp = apollo::common::time::ToSecond(Clock::Now());
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "after optimizer " << optimizer->name() << ":"
           << planning_data->DebugString();

    if (FLAGS_enable_record_debug && ptr_debug != nullptr &&
        ptr_latency_stats != nullptr) {
      RecordDebugInfo(optimizer->name(), planning_data, time_diff_ms, ptr_debug,
                      ptr_latency_stats);
    }
  }
  if (!planning_data->CombinePathAndSpeedProfile(
      FLAGS_output_trajectory_time_resolution,
      planning_start_point.relative_time(),
      ptr_trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  PopulateDecision(frame);

  // Add debug information.
  if (FLAGS_enable_record_debug && ptr_debug != nullptr) {
    auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
    reference_line->set_name("planning_reference_line");
    const auto& reference_points = frame->reference_line().reference_points();
    reference_line->mutable_path_point()->CopyFrom(
        {reference_points.begin(), reference_points.end()});
  }
  return Status::OK();
}

std::vector<SpeedPoint> EMPlanner::GenerateInitSpeedProfile(
    const common::TrajectoryPoint& planning_init_point) {
  std::vector<SpeedPoint> speed_profile;
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (last_frame != nullptr) {
    const auto& last_speed_vector =
          last_frame->planning_data().speed_data().speed_vector();

    if (!last_speed_vector.empty()) {
      const auto& last_init_point =
          last_frame->PlanningStartPoint().path_point();
      Vec2d last_xy_point = {last_init_point.x(), last_init_point.y()};
      SLPoint last_sl_point;
      if (!last_frame->reference_line().
            get_point_in_frenet_frame(last_xy_point, &last_sl_point)) {
          AERROR << "Fail to transfer xy to sl when init speed profile";
      }
      Vec2d xy_point = {planning_init_point.path_point().x(),
                                      planning_init_point.path_point().y()};
      SLPoint sl_point;
      if (!last_frame->reference_line().
            get_point_in_frenet_frame(xy_point, &last_sl_point)) {
          AERROR << "Fail to transfer xy to sl when init speed profile";
      }
      double s_diff = sl_point.s() - last_sl_point.s();
      double start_time = 0.0;
      double start_s = 0.0;
      bool is_updated_start = false;
      for (const auto& speed_point : last_speed_vector) {
            if (speed_point.s() < s_diff) {
                continue;
            }
            if (!is_updated_start) {
                start_time = speed_point.t();
                start_s = speed_point.s();
                is_updated_start = true;
            }
            SpeedPoint refined_speed_point;
            refined_speed_point.set_s(speed_point.s() - start_s);
            refined_speed_point.set_t(speed_point.t() - start_time);
            refined_speed_point.set_v(speed_point.v());
            refined_speed_point.set_a(speed_point.a());
            refined_speed_point.set_da(speed_point.da());
            speed_profile.push_back(refined_speed_point);
      }
    }
  }
  if (speed_profile.empty()) {
    // This is a dummy simple hot start, need refine later
    std::array<double, 3> start_state;

    // distance 0.0
    start_state[0] = 0.0;

    // start velocity
    start_state[1] = planning_init_point.v();

    // start acceleration
    start_state[2] = planning_init_point.a();

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
                                   FLAGS_trajectory_time_resolution) + 1;
    speed_profile.reserve(num_time_steps);

    for (std::uint32_t i = 0; i < num_time_steps; ++i) {
      double t = i * FLAGS_trajectory_time_resolution;
      double s = speed_curve.Evaluate(0, t);
      double v = speed_curve.Evaluate(1, t);
      double a = speed_curve.Evaluate(2, t);
      double da = speed_curve.Evaluate(3, t);
      SpeedPoint speed_point;
      speed_point.set_s(s);
      speed_point.set_t(t);
      speed_point.set_v(v);
      speed_point.set_a(a);
      speed_point.set_da(da);
      speed_profile.push_back(std::move(speed_point));
    }
  }
  return speed_profile;
}

}  // namespace planning
}  // namespace apollo
