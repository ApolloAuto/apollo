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
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::hdmap::PathOverlap;

constexpr double kRoadBuffer = 0.2;
constexpr double kObstacleBuffer = 0.2;
constexpr double kPlanDistAfterObs = 3.0;

SidePassPathDecider::SidePassPathDecider(const TaskConfig &config)
    : Decider(config) {
  fem_qp_.reset(new Fem1dExpandedJerkQpProblem());
  delta_s_ = config.side_pass_path_decider_config().path_resolution();
  const int n = static_cast<int>(
      config.side_pass_path_decider_config().total_path_length() / delta_s_);
  std::array<double, 3> l_init = {0.0, 0.0, 0.0};
  std::array<double, 5> w = {
      config.side_pass_path_decider_config().l_weight(),
      config.side_pass_path_decider_config().dl_weight(),
      config.side_pass_path_decider_config().ddl_weight(),
      config.side_pass_path_decider_config().dddl_weight(),
      config.side_pass_path_decider_config().guiding_line_weight(),
  };
  CHECK(fem_qp_->Init(n, l_init, delta_s_, w,
                      config.side_pass_path_decider_config().max_dddl()));
}

Status SidePassPathDecider::Process(Frame *frame,
                                    ReferenceLineInfo *reference_line_info) {
  GeneratePath(frame, reference_line_info);
  return Status::OK();
}

Status SidePassPathDecider::BuildSidePathDecision(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  // TODO(All): decide side pass from left or right.
  // For now, just go left at all times.
  decided_direction_ = SidePassDirection::LEFT;
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

  // TODO(All): Check if ADC has fully stopped.

  // Decide whether to side-pass from left or right.
  if (BuildSidePathDecision(frame, reference_line_info) != Status().OK()) {
    AERROR << "Failed to decide on a side-pass direction.";
    return false;
  }

  auto list_s_leftbound_rightbound = GetPathBoundaries(
      frame->PlanningStartPoint(), reference_line_info->AdcSlBoundary(),
      reference_line_info->reference_line(),
      reference_line_info->path_decision()->obstacles());

  for (const auto tp : list_s_leftbound_rightbound) {
    ADEBUG << std::get<0>(tp) << ", " << std::get<1>(tp) << ", "
           << std::get<2>(tp);
  }

  // Call optimizer to generate smooth path.
  fem_qp_->SetVariableBounds(list_s_leftbound_rightbound);
  if (!fem_qp_->Optimize()) {
    AERROR << "Fail to optimize in SidePassPathDecider.";
    return false;
  }

  // TODO(All): put optimized results into ReferenceLineInfo.
  // Update Reference_Line_Info with this newly generated path.
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  auto adc_frenet_frame_point =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint());
  double accumulated_s = adc_frenet_frame_point.s();
  for (size_t i = 0; i < fem_qp_->x().size(); ++i) {
    common::FrenetFramePoint frenet_frame_point;
    ADEBUG << "FrenetFramePath: s = " << accumulated_s
           << ", l = " << fem_qp_->x()[i];
    if (accumulated_s >= reference_line_info->reference_line().Length()) {
      break;
    }
    frenet_frame_point.set_s(accumulated_s);
    frenet_frame_point.set_l(fem_qp_->x()[i]);
    frenet_frame_point.set_dl(fem_qp_->x_derivative()[i]);
    frenet_frame_point.set_ddl(fem_qp_->x_second_order_derivative()[i]);
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    accumulated_s += delta_s_;
  }

  auto path_data = reference_line_info->mutable_path_data();
  path_data->SetReferenceLine(&reference_line_info->reference_line());
  path_data->SetFrenetPath(FrenetFramePath(frenet_frame_path));

  return true;
}

std::vector<std::tuple<double, double, double>>
SidePassPathDecider::GetPathBoundaries(
    const TrajectoryPoint &planning_start_point,
    const SLBoundary &adc_sl_boundary, const ReferenceLine &reference_line,
    const IndexedList<std::string, Obstacle> &indexed_obstacles) {
  std::vector<std::tuple<double, double, double>> list_s_leftbound_rightbound;

  const auto nearest_obs_sl_boundary =
      GetNearestObstacle(adc_sl_boundary, reference_line, indexed_obstacles)
          ->PerceptionSLBoundary();

  // Get road info at every point and fill in the boundary condition vector.
  common::PathPoint start_path_point = planning_start_point.path_point();
  common::math::Vec2d start_path_point_vec2d(start_path_point.x(),
                                             start_path_point.y());
  common::SLPoint start_path_point_sl;
  if (!reference_line.XYToSL(start_path_point_vec2d, &start_path_point_sl)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return list_s_leftbound_rightbound;
  }
  double s_increment = 1.0;
  double curr_s = start_path_point_sl.s();
  bool bound_cond_gen_finished = false;
  bool is_blocked_by_obs = false;

  // Currently, it only considers one obstacle.
  // For future scaling so that multiple obstacles can be considered,
  // a sweep-line method can be used. The code here leaves some room
  // for the sweep-line method.
  while (!bound_cond_gen_finished) {
    std::tuple<double, double, double> s_leftbound_rightbound;
    std::get<0>(s_leftbound_rightbound) = curr_s;
    // Check if boundary should be dictated by obstacle or road
    if (curr_s >= nearest_obs_sl_boundary.start_s() - kObstacleBuffer &&
        curr_s <= nearest_obs_sl_boundary.end_s() + kObstacleBuffer) {
      is_blocked_by_obs = true;
    } else {
      is_blocked_by_obs = false;
    }
    // Get the road info at the current s.
    double road_left_width_at_curr_s = 0.0;
    double road_right_width_at_curr_s = 0.0;
    reference_line.GetRoadWidth(curr_s, &road_left_width_at_curr_s,
                                &road_right_width_at_curr_s);
    if (!is_blocked_by_obs) {
      std::get<1>(s_leftbound_rightbound) =
          -std::abs(road_left_width_at_curr_s - kRoadBuffer);
      std::get<2>(s_leftbound_rightbound) =
          std::abs(road_right_width_at_curr_s - kRoadBuffer);
    } else {
      if (decided_direction_ == SidePassDirection::LEFT) {
        std::get<1>(s_leftbound_rightbound) =
            -std::abs(road_left_width_at_curr_s - kRoadBuffer);
        std::get<2>(s_leftbound_rightbound) =
            -nearest_obs_sl_boundary.end_l() - kObstacleBuffer;
      } else if (decided_direction_ == SidePassDirection::RIGHT) {
        std::get<1>(s_leftbound_rightbound) =
            -nearest_obs_sl_boundary.start_l() + kObstacleBuffer;
        std::get<2>(s_leftbound_rightbound) =
            std::abs(road_right_width_at_curr_s - kRoadBuffer);
      } else {
        AERROR << "Side-pass direction undefined.";
        return list_s_leftbound_rightbound;
      }
    }
    list_s_leftbound_rightbound.push_back(s_leftbound_rightbound);
    // Move to next s
    curr_s += s_increment;
    if (curr_s > kPlanDistAfterObs + nearest_obs_sl_boundary.end_s()) {
      bound_cond_gen_finished = true;
    }
  }

  return list_s_leftbound_rightbound;
}

const Obstacle *SidePassPathDecider::GetNearestObstacle(
    const SLBoundary &adc_sl_boundary, const ReferenceLine &reference_line,
    const IndexedList<std::string, Obstacle> &indexed_obstacles) {
  const Obstacle *nearest_obstacle = nullptr;

  // Generate the boundary conditions for the selected direction
  // based on the obstacle ahead and road conditions.
  double adc_end_s = adc_sl_boundary.end_s();

  // Get obstacle info.
  bool no_obs_selected = true;
  double nearest_obs_start_s = 0.0;
  for (const auto *obstacle : indexed_obstacles.Items()) {
    // Filter out obstacles that are behind ADC.
    double obs_start_s = obstacle->PerceptionSLBoundary().start_s();
    double obs_end_s = obstacle->PerceptionSLBoundary().end_s();
    if (obs_end_s < adc_end_s) {
      continue;
    }
    // TODO(All): ignores obstacles that are partially ahead of ADC
    if (obs_start_s < adc_end_s) {
      continue;
    }
    // Filter out those out-of-lane obstacles.
    double lane_left_width_at_start_s = 0.0;
    double lane_left_width_at_end_s = 0.0;
    double lane_right_width_at_start_s = 0.0;
    double lane_right_width_at_end_s = 0.0;
    reference_line.GetLaneWidth(obs_start_s, &lane_left_width_at_start_s,
                                &lane_right_width_at_start_s);
    reference_line.GetLaneWidth(obs_end_s, &lane_left_width_at_end_s,
                                &lane_right_width_at_end_s);
    double lane_left_width = std::min(std::abs(lane_left_width_at_start_s),
                                      std::abs(lane_left_width_at_end_s));
    double lane_right_width = std::min(std::abs(lane_right_width_at_start_s),
                                       std::abs(lane_right_width_at_end_s));
    double obs_start_l = obstacle->PerceptionSLBoundary().start_l();
    double obs_end_l = obstacle->PerceptionSLBoundary().end_l();

    // filter out-of-lane obstacles
    if (obs_start_l > lane_left_width || obs_end_l < -lane_right_width) {
      continue;
    }
    // do NOT sidepass non-vehicle obstacles.
    if (obstacle->Perception().type() !=
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
      nearest_obstacle = obstacle;
      no_obs_selected = false;
    }
    if (nearest_obs_start_s > obs_start_s) {
      nearest_obs_start_s = obs_start_s;
    }
  }
  return nearest_obstacle;
}

}  // namespace planning
}  // namespace apollo
