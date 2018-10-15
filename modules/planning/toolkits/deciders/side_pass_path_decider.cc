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

#include "modules/planning/toolkits/deciders/side_pass_path_decider.h"

#include <algorithm>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using ::apollo::common::ErrorCode;
using ::apollo::common::Status;
using ::apollo::hdmap::PathOverlap;

double ROAD_BUFFER = 0.2;
double OBSTACLE_BUFFER = 0.2;
double PLANNING_DIST_AFTER_OBSTACLE = 3.0;

SidePassPathDecider::SidePassPathDecider(const TaskConfig &config)
    : Decider(config) {
  SetName("SidePassPathDecider");
  fem_qp_.reset(new Fem1dExpandedJerkQpProblem());
  // TODO(lianglia-apollo):
  // Put numbers into config when refactor is finished.
  const int n = 400;
  std::array<double, 3> l_init = {0.0, 0.0, 0.0};
  const double delta_s = 0.5;
  std::array<double, 5> w = {1.0, 2.0, 3.0, 4.0, 5.0};
  constexpr double kMaxLThirdOrderDerivative = 10.0;
  CHECK(fem_qp_->Init(n, l_init, delta_s, w, kMaxLThirdOrderDerivative));
}

Status SidePassPathDecider::Process(Frame *frame,
                                    ReferenceLineInfo *reference_line_info) {
  return Status::OK();
}

Status SidePassPathDecider::BuildSidePathDecision(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  // TODO(All): decide side pass from left or right.
  // For now, just go left at all times.
  decided_direction_ = LEFT;
  return Status().OK();
}

// TODO(All): currently it's the 1st version, and only consider one
// vehicular obstacle ahead. It side-passes that obstacle and move
// back to original reference_line immediately. (without considering
// subsequent obstacles)
bool SidePassPathDecider::GeneratePath(Frame *frame,
                                       ReferenceLineInfo *reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Check if ADC has fully stopped.
  // TODO(All)

  // Decide whether to side-pass from left or right.
  if (BuildSidePathDecision(frame, reference_line_info) != Status().OK()) {
    AERROR << "Failed to decide on a side-pass direction.";
    return false;
  }

  // Generate the boundary conditions for the selected direction
  // based on the obstacle ahead and road conditions.
  std::vector<std::pair<double, double>> bound_cond;
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  // Get obstacle info.
  bool no_obs_selected = true;
  double nearest_obs_start_s, nearest_obs_end_s,
         nearest_obs_start_l, nearest_obs_end_l;
  for (const auto* path_obstacle :
       reference_line_info->path_decision()->path_obstacles().Items()) {
    // Filter out obstacles that are behind ADC.
    double obs_start_s = path_obstacle->PerceptionSLBoundary().start_s();
    double obs_end_s = path_obstacle->PerceptionSLBoundary().end_s();
    if (obs_end_s < adc_end_s) {
      continue;
    }
    // TODO(All): currently, it also ignores obstacles that are
    // partially ahead of ADC, meaning (obs_end_s > adc_end_s) but
    // (obs_start_s < adc_end_s). It treats such obstacles as
    // those that have been considered already in the last cycle.
    if (obs_start_s < adc_end_s) {
      continue;
    }
    // Filter out those out-of-lane obstacles.
    double lane_left_width_at_start_s, lane_left_width_at_end_s,
           lane_right_width_at_start_s, lane_right_width_at_end_s;
    reference_line.GetLaneWidth(obs_start_s,
        &lane_left_width_at_start_s, &lane_right_width_at_start_s);
    reference_line.GetLaneWidth(obs_end_s,
        &lane_left_width_at_end_s, &lane_right_width_at_end_s);
    double lane_left_width = std::min(std::abs(lane_left_width_at_start_s),
                                      std::abs(lane_left_width_at_end_s));
    double lane_right_width = std::min(std::abs(lane_right_width_at_start_s),
                                      std::abs(lane_right_width_at_end_s));
    double obs_start_l = path_obstacle->PerceptionSLBoundary().start_l();
    double obs_end_l = path_obstacle->PerceptionSLBoundary().end_l();
    if (obs_start_l > lane_left_width || -obs_end_l > lane_right_width) {
      continue;
    }
    // Only select vehicle obstacles.
    if (path_obstacle->obstacle()->Perception().type() !=
        perception::PerceptionObstacle::VEHICLE) {
      continue;
    }
    // For obstacles of interests, select the nearest one.
    // TODO(All): currently, regardless of the orientation
    // of the obstacle, it treats the obstacle as a rectangle
    // with two edges parallel to the reference line and the
    // other two perpendicular to that.
    if (no_obs_selected) {
      nearest_obs_start_s = obs_start_s;
      nearest_obs_end_s = obs_end_s;
      nearest_obs_start_l = obs_start_l;
      nearest_obs_end_l = obs_end_l;
      no_obs_selected = false;
    }
    if (nearest_obs_start_s > obs_start_s) {
      nearest_obs_start_s = obs_start_s;
      nearest_obs_end_s = obs_end_s;
      nearest_obs_start_l = obs_start_l;
      nearest_obs_end_l = obs_end_l;
    }
  }
  // Get road info at every point and fill in the boundary condition vector.
  common::PathPoint start_path_point = frame->PlanningStartPoint().path_point();
  common::math::Vec2d start_path_point_vec2d(start_path_point.x(),
                                             start_path_point.y());
  common::SLPoint start_path_point_SL;
  if (!reference_line.XYToSL(start_path_point_vec2d, &start_path_point_SL)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return false;
  }
  double s_increment = 1.0;
  double curr_s = start_path_point_SL.s();
  bool bound_cond_gen_finished = false;
  bool is_blocked_by_obs = false;
  std::vector<std::vector<double>> list_s_leftbound_rightbound;
  // Currently, it only considers one obstacle.
  // For future scaling so that multiple obstacles can be considered,
  // a sweep-line method can be used. The code here leaves some room
  // for the sweep-line method.
  while (!bound_cond_gen_finished) {
    std::vector<double> s_leftbound_rightbound;
    s_leftbound_rightbound.push_back(curr_s);
    // Check if boundary should be dictated by obstacle or road
    if (curr_s >= nearest_obs_start_s && curr_s <= nearest_obs_end_s) {
      is_blocked_by_obs = true;
    } else {
      is_blocked_by_obs = false;
    }
    // Get the road info at the current s.
    double road_left_width_at_curr_s, road_right_width_at_curr_s;
    reference_line.GetRoadWidth(curr_s,
        &road_left_width_at_curr_s, &road_right_width_at_curr_s);
    if (!is_blocked_by_obs) {
      s_leftbound_rightbound.push_back
          (-std::abs(road_left_width_at_curr_s-ROAD_BUFFER));
      s_leftbound_rightbound.push_back
          (std::abs(road_right_width_at_curr_s-ROAD_BUFFER));
    } else {
      if (decided_direction_ == LEFT) {
        s_leftbound_rightbound.push_back
            (-std::abs(road_left_width_at_curr_s-ROAD_BUFFER));
        s_leftbound_rightbound.push_back
            (-nearest_obs_end_l);
      } else if (decided_direction_ == RIGHT) {
        s_leftbound_rightbound.push_back
            (-nearest_obs_start_l);
        s_leftbound_rightbound.push_back
            (std::abs(road_right_width_at_curr_s-ROAD_BUFFER));
      } else {
        AERROR << "Side-pass direction undefined.";
        return false;
      }
    }
    // Move to next s
    curr_s += s_increment;
    if (curr_s > PLANNING_DIST_AFTER_OBSTACLE + nearest_obs_end_s) {
      bound_cond_gen_finished = true;
    }
  }
  // Call optimizer: (name to be filled) to generate smooth path.

  // Update Reference_Line_Info with this newly generated path.

  // TODO(All): generate path here

  std::vector<std::tuple<double, double, double>> l_bounds;

  // TODO(All): set up l_bounds here.
  fem_qp_->SetVariableBounds(l_bounds);
  fem_qp_->Optimize();
  // TODO(All): put optimized results into ReferenceLineInfo.



  return true;
}

}  // namespace planning
}  // namespace apollo
