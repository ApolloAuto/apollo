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

/**
 * @file speed_optimizer.cc
 **/

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/tasks/speed_optimizer.h"

namespace apollo {
namespace planning {

using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;
using apollo::common::Status;

namespace {
constexpr double kSpeedOptimizationFallbackClost = 2e4;
}

SpeedOptimizer::SpeedOptimizer(const std::string& name) : Task(name) {}

apollo::common::Status SpeedOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);

  auto ret = Process(
      reference_line_info->AdcSlBoundary(), reference_line_info->path_data(),
      frame->PlanningStartPoint(), reference_line_info->reference_line(),
      *reference_line_info->mutable_speed_data(),
      reference_line_info->path_decision(),
      reference_line_info->mutable_speed_data());

  if (!ret.ok() && FLAGS_enable_slowdown_profile_generator) {
    *reference_line_info->mutable_speed_data() = GenerateStopProfile(
        frame->PlanningStartPoint().v(), frame->PlanningStartPoint().a());
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost);
    ret = Status::OK();
  }
  RecordDebugInfo(reference_line_info->speed_data());
  return ret;
}

SpeedData SpeedOptimizer::GenerateStopProfile(const double init_speed,
                                              const double init_acc) const {
  AERROR << "Slowing down the car.";
  SpeedData speed_data;

  const double kFixedJerk = -1.0;
  const double first_point_acc = std::fmin(0.0, init_acc);

  const size_t max_t = 3.0;
  const double unit_t = 0.02;

  double pre_s = 0.0;
  const double t_mid =
      (FLAGS_slowdown_profile_deceleration - first_point_acc) / kFixedJerk;
  const double s_mid = init_speed * t_mid +
                       0.5 * first_point_acc * t_mid * t_mid +
                       1.0 / 6.0 * kFixedJerk * t_mid * t_mid * t_mid;
  const double v_mid =
      init_speed + first_point_acc * t_mid + 0.5 * kFixedJerk * t_mid * t_mid;

  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    if (t <= t_mid) {
      s = std::fmax(pre_s, init_speed * t + 0.5 * first_point_acc * t * t +
                               1.0 / 6.0 * kFixedJerk * t * t * t);
      v = std::fmax(
          0.0, init_speed + first_point_acc * t + 0.5 * kFixedJerk * t * t);
      const double a = first_point_acc + kFixedJerk * t;
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      pre_s = s;
    } else {
      s = std::fmax(pre_s, s_mid + v_mid * (t - t_mid) +
                               0.5 * FLAGS_slowdown_profile_deceleration *
                                   (t - t_mid) * (t - t_mid));
      v = std::fmax(0.0,
                    v_mid + (t - t_mid) * FLAGS_slowdown_profile_deceleration);
      speed_data.AppendSpeedPoint(s, t, v, FLAGS_slowdown_profile_deceleration,
                                  0.0);
    }
    pre_s = s;
  }
  return speed_data;
}

void SpeedOptimizer::RecordDebugInfo(const SpeedData& speed_data) {
  auto* debug = reference_line_info_->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name(Name());
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

void SpeedOptimizer::RecordSTGraphDebug(const StGraphData& st_graph_data,
                                        STGraphDebug* st_graph_debug) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  st_graph_debug->set_name(Name());
  for (const auto& boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case StBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case StBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case StBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case StBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case StBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case StBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto& point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto& point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint speed_point;
    speed_point.set_s(point.first);
    speed_point.set_v(point.second);
    st_graph_debug->add_speed_limit()->CopyFrom(speed_point);
  }

  const auto& speed_data = reference_line_info_->speed_data();
  st_graph_debug->mutable_speed_profile()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

}  // namespace planning
}  // namespace apollo
