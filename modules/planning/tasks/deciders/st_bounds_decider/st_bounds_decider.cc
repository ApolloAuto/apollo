/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"

#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

namespace {
// STBoundPoint contains (t, s_min, s_max)
using STBoundPoint = std::tuple<double, double, double>;
// STBound is a vector of STBoundPoints
using STBound = std::vector<STBoundPoint>;
}  // namespace

STBoundsDecider::STBoundsDecider(const TaskConfig& config) : Decider(config) {
  CHECK(config.has_st_bounds_decider_config());
  st_bounds_config_ = config.st_bounds_decider_config();
}

Status STBoundsDecider::Process(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  // Initialize the related helper classes.
  InitSTBoundsDecider(*frame, reference_line_info);

  // Sweep the t-axis, and determine the s-boundaries step by step.
  STBound regular_st_bound;
  Status ret = GenerateRegularSTBound(&regular_st_bound);
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a regular ST-boundary.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (regular_st_bound.empty()) {
    std::string msg = "Generated regular ST-boundary is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  return Status::OK();
}

void STBoundsDecider::InitSTBoundsDecider(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  const PathData& path_data = reference_line_info->path_data();
  PathDecision* path_decision = reference_line_info->path_decision();

  // Map all related obstacles onto ST-Graph.
  auto time1 = std::chrono::system_clock::now();
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               st_bounds_config_.total_time(), path_data);
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";
  // Record the ST-Graph for good visualization and easy debugging.
  auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  for (auto it = all_st_boundaries.begin(); it != all_st_boundaries.end();
       ++it) {
    st_boundaries.push_back(it->second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info_->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, st_graph_debug);

  // Initialize Guide-Line and Driving-Limits.
  constexpr double desired_speed = 15.0;
  st_guide_line_.Init(desired_speed);
  constexpr double max_acc = 2.0;
  constexpr double max_dec = 4.0;
  constexpr double max_v = desired_speed * 1.5;
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.PlanningStartPoint().v());
}

Status STBoundsDecider::GenerateRegularSTBound(STBound* const st_bound) {
  // Initialize st-boundary.
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();
       curr_t += kSTBoundsDeciderResolution) {
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t, s_lower, s_upper;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);

    // Get Boundary due to driving limits
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);

    // Get Boundary due to obstacles
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<std::vector<std::pair<std::string, ObjectDecisionType>>>
        available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);

    if (available_s_bounds.size() >= 1) {
      RankDecisions(st_guide_line_.GetGuideSFromT(t), driving_limits_bound,
                    &available_s_bounds, &available_obs_decisions);
      s_lower = std::fmax(s_lower, available_s_bounds.front().first);
      s_upper = std::fmin(s_upper, available_s_bounds.front().second);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
  }

  return Status::OK();
}

void STBoundsDecider::RankDecisions(
    double s_guide_line, std::pair<double, double> driving_limit,
    std::vector<std::pair<double, double>>* const available_s_bounds,
    std::vector<std::vector<std::pair<std::string, ObjectDecisionType>>>* const
        available_obs_decisions) {
  // TODO(jiacheng): implement this.
  return;
}

void STBoundsDecider::RecordSTGraphDebug(
    const std::vector<STBoundary>& st_graph_data,
    planning_internal::STGraphDebug* const st_graph_debug) {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto& boundary : st_graph_data) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    ADEBUG << "Obstacle ID = " << boundary.id();
    boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);

    for (const auto& point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }
}

}  // namespace planning
}  // namespace apollo
