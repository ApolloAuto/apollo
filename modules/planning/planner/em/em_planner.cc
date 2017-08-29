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
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/planner/em/decider.h"
#include "modules/planning/tasks/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/tasks/path_decider/path_decider.h"
#include "modules/planning/tasks/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/tasks/traffic_decider/traffic_decider.h"

namespace apollo {
namespace planning {

using common::Status;
using common::adapter::AdapterManager;
using common::time::Clock;
using common::ErrorCode;
using common::SpeedPoint;
using common::SLPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

void EMPlanner::RegisterTasks() {
  task_factory_.Register(TRAFFIC_DECIDER,
                         []() -> Task* { return new TrafficDecider(); });
  task_factory_.Register(DP_POLY_PATH_OPTIMIZER,
                         []() -> Task* { return new DpPolyPathOptimizer(); });
  task_factory_.Register(PATH_DECIDER,
                         []() -> Task* { return new PathDecider(); });
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new DpStSpeedOptimizer(); });
  task_factory_.Register(QP_SPLINE_PATH_OPTIMIZER,
                         []() -> Task* { return new QpSplinePathOptimizer(); });
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Task* {
    return new QpSplineStSpeedOptimizer();
  });
}

Status EMPlanner::Init(const PlanningConfig& config) {
  AINFO << "In EMPlanner::Init()";
  RegisterTasks();
  for (const auto task : config.em_planner_config().task()) {
    tasks_.emplace_back(
        task_factory_.CreateObject(static_cast<TaskType>(task)));
    AINFO << "Created task:" << tasks_.back()->Name();
  }
  for (auto& task : tasks_) {
    if (!task->Init(config)) {
      std::string msg(
          common::util::StrCat("Init task[", task->Name(), "] failed."));
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

void EMPlanner::RecordDebugInfo(const std::string& name,
                                const double time_diff_ms,
                                planning::LatencyStats* ptr_latency_stats) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

void EMPlanner::PopulateDecision(const ReferenceLineInfo& reference_line_info,
                                 Frame* frame) {
  auto* planning_pb = frame->MutableADCTrajectory();
  Decider decider;
  planning_pb->mutable_decision()->CopyFrom(
      decider.MakeDecision(reference_line_info));
}

Status EMPlanner::Plan(const TrajectoryPoint& planning_start_point,
                       Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!frame) {
    AERROR << "Frame is empty in EMPlanner";
    return Status(ErrorCode::PLANNING_ERROR, "Frame is null");
  }

  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile =
      GenerateInitSpeedProfile(planning_start_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile = GenerateSpeedHotStart(planning_start_point);
    AINFO << "Using dummy hot start for speed vector";
  }
  heuristic_speed_data->set_speed_vector(speed_profile);

  auto ptr_debug = reference_line_info->mutable_debug();
  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();
  for (auto& optimizer : tasks_) {
    const double start_timestamp = Clock::NowInSecond();
    auto ret = optimizer->Execute(frame, reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << optimizer->Name()
             << "], Error message: " << ret.error_message();
      return ret;
    }
    const double end_timestamp = Clock::NowInSecond();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "after optimizer " << optimizer->Name() << ":"
           << reference_line_info->PathSpeedDebugString() << std::endl;
    ADEBUG << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";

    if (FLAGS_enable_record_debug && ptr_debug != nullptr &&
        ptr_latency_stats != nullptr) {
      RecordDebugInfo(optimizer->Name(), time_diff_ms, ptr_latency_stats);
    }
  }
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          FLAGS_output_trajectory_time_resolution,
          planning_start_point.relative_time(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  reference_line_info->SetTrajectory(trajectory);
  PopulateDecision(*reference_line_info, frame);

  return Status::OK();
}

std::vector<SpeedPoint> EMPlanner::GenerateInitSpeedProfile(
    const common::TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) {
  std::vector<SpeedPoint> speed_profile;
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    AWARN << "last frame is empty";
    return speed_profile;
  }
  const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLinfInfo();
  if (!last_reference_line_info) {
    AINFO << "last reference line info is empty";
    return speed_profile;
  }
  if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {
    AINFO << "Current reference line is not started previous drived line";
    return speed_profile;
  }
  const auto& last_speed_vector =
      last_reference_line_info->speed_data().speed_vector();

  if (!last_speed_vector.empty()) {
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point();
    Vec2d last_xy_point(last_init_point.x(), last_init_point.y());
    SLPoint last_sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point,
                                                           &last_sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    Vec2d xy_point(planning_init_point.path_point().x(),
                   planning_init_point.path_point().y());
    SLPoint sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(xy_point,
                                                           &last_sl_point)) {
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
      speed_profile.push_back(std::move(refined_speed_point));
    }
  }
  return speed_profile;
}

// This is a dummy simple hot start, need refine later
std::vector<common::SpeedPoint> EMPlanner::GenerateSpeedHotStart(
    const common::TrajectoryPoint& planning_init_point) {
  std::array<double, 3> start_state;

  // distance 0.0
  start_state[0] = 0.0;

  // start velocity
  start_state[1] = planning_init_point.v();

  // start acceleration
  start_state[2] = planning_init_point.a();

  std::array<double, 2> end_state;
  // end state velocity
  end_state[0] = std::max(10.0, planning_init_point.v());

  // end state acceleration
  end_state[1] = 0.0;

  QuarticPolynomialCurve1d speed_profile(start_state, end_state,
                                         FLAGS_trajectory_time_length);
  // assume the time resolution is 0.1
  std::uint32_t num_time_steps =
      static_cast<std::uint32_t>(FLAGS_trajectory_time_length /
                                 FLAGS_trajectory_time_resolution) +
      1;

  std::vector<common::SpeedPoint> discretized_speed_profile;
  discretized_speed_profile.reserve(num_time_steps);
  for (std::uint32_t i = 0; i < num_time_steps; ++i) {
    double t = i * FLAGS_trajectory_time_resolution;
    double s = speed_profile.Evaluate(0, t);
    double v = speed_profile.Evaluate(1, t);
    double a = speed_profile.Evaluate(2, t);
    double da = speed_profile.Evaluate(3, t);
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_point.set_da(da);
    discretized_speed_profile.push_back(std::move(speed_point));
  }
  return discretized_speed_profile;
}

}  // namespace planning
}  // namespace apollo
