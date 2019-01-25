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

#include "modules/planning/tasks/deciders/side_pass_path_decider.h"

#include <algorithm>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;

SidePassPathDecider::SidePassPathDecider(const TaskConfig &config)
    : Decider(config) {}

void SidePassPathDecider::InitSolver() {
  const TaskConfig &config = Decider::config_;
  const int n = static_cast<int>(total_path_length_ / delta_s_);
  std::array<double, 3> l_init = {adc_frenet_frame_point_.l(),
                                  adc_frenet_frame_point_.dl(),
                                  adc_frenet_frame_point_.ddl()};
  std::array<double, 5> w = {
      config.side_pass_path_decider_config().l_weight(),
      config.side_pass_path_decider_config().dl_weight(),
      config.side_pass_path_decider_config().ddl_weight(),
      config.side_pass_path_decider_config().dddl_weight(),
      config.side_pass_path_decider_config().guiding_line_weight(),
  };
  fem_qp_.reset(new Fem1dExpandedJerkQpProblem());
  CHECK(fem_qp_->Init(n, l_init, delta_s_, w,
                      config.side_pass_path_decider_config().max_dddl()));
}

Status SidePassPathDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  adc_planning_start_point_ = frame->PlanningStartPoint();
  adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());
  delta_s_ = Decider::config_.side_pass_path_decider_config().path_resolution();
  if (std::fabs(delta_s_) < 1e-6) {
    const std::string msg = "delta_s_ is too small.";
    AERROR << msg << ", delta_s_ = " << delta_s_;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  total_path_length_ =
      Decider::config_.side_pass_path_decider_config().total_path_length();

  nearest_obstacle_ =
      GetNearestObstacle(reference_line_info->AdcSlBoundary(),
                         reference_line_info->reference_line(),
                         reference_line_info->path_decision()->obstacles());
  if (nearest_obstacle_ == nullptr) {
    const std::string msg = "Fail to get nearest obstacle.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!GeneratePath(frame, reference_line_info)) {
    const std::string msg = "Fail to generate path.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

bool SidePassPathDecider::DecideSidePassDirection(
    const std::vector<bool> &can_side_pass, size_t left_length,
    size_t right_length) {
  if (can_side_pass[0] == false && can_side_pass[1] == false) {
    return false;
  } else if (can_side_pass[0] == true && can_side_pass[1] == false) {
    decided_direction_ = SidePassDirection::LEFT;
    return true;
  } else if (can_side_pass[0] == false && can_side_pass[1] == true) {
    decided_direction_ = SidePassDirection::RIGHT;
    return true;
  } else {
    if (curr_lane_.left_neighbor_forward_lane_id_size() > 0) {
      decided_direction_ = SidePassDirection::LEFT;
    } else if (curr_lane_.right_neighbor_forward_lane_id_size() > 0) {
      decided_direction_ = SidePassDirection::RIGHT;
    } else if (curr_lane_.left_neighbor_reverse_lane_id_size() > 0) {
      decided_direction_ = SidePassDirection::LEFT;
    } else if (curr_lane_.right_neighbor_reverse_lane_id_size() > 0) {
      decided_direction_ = SidePassDirection::RIGHT;
    } else {
      if (left_length <= right_length) {
        decided_direction_ = SidePassDirection::LEFT;
      } else {
        decided_direction_ = SidePassDirection::RIGHT;
      }
    }
    return true;
  }
}

bool SidePassPathDecider::GetLaneInfoFromPoint(
    double point_x, double point_y, double point_z, double point_theta,
    hdmap::LaneInfoConstPtr* const lane) {
  constexpr double kLaneSearchRadius = 1.0;
  constexpr double kLaneSearchMaxThetaDiff = M_PI / 3.0;
  double s = 0.0;
  double l = 0.0;
  if (HDMapUtil::BaseMapPtr()->GetNearestLaneWithHeading(
          common::util::MakePointENU(point_x, point_y, point_z),
          kLaneSearchRadius, point_theta, kLaneSearchMaxThetaDiff,
          lane, &s, &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << "(x, y, z) = (" << point_x << ", " << point_y << ", "
           << point_z << ")" << ", heading = " << point_theta;
    return false;
  }
  return true;
}

bool SidePassPathDecider::GeneratePath(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Get the current LaneInfo of the ADC.
  hdmap::LaneInfoConstPtr lane;
  if (!GetLaneInfoFromPoint(
          adc_planning_start_point_.path_point().x(),
          adc_planning_start_point_.path_point().y(),
          adc_planning_start_point_.path_point().z(),
          adc_planning_start_point_.path_point().theta(),
          &lane)) {
    return false;
  }
  curr_lane_ = lane->lane();
  ADEBUG << curr_lane_.ShortDebugString();

  // Try generating paths for both directions.
  std::vector<SidePassDirection> side_pass_directions = {
      SidePassDirection::LEFT, SidePassDirection::RIGHT};
  std::vector<bool> can_side_pass(2, true);
  std::vector<std::vector<common::FrenetFramePoint>> frenet_frame_paths(
      2, std::vector<common::FrenetFramePoint>(0));
  for (size_t i = 0; i < side_pass_directions.size(); i++) {
    // Pick a direction and try generating a path.
    decided_direction_ = side_pass_directions[i];
    if (decided_direction_ == SidePassDirection::LEFT) {
      ADEBUG << "\n";
      ADEBUG << "Trying to side-pass from left:";
    } else {
      ADEBUG << "\n";
      ADEBUG << "Trying to side-pass from right:";
    }
    // 1. Generate boundary
    bool fail_to_find_boundary = false;
    auto lateral_bounds = GetPathBoundaries(
        frame->PlanningStartPoint(), reference_line_info->AdcSlBoundary(),
        reference_line_info->reference_line(),
        reference_line_info->path_decision()->obstacles(),
        &fail_to_find_boundary);
    if (fail_to_find_boundary) {
      can_side_pass[i] = false;
      ADEBUG << "  - Failed to find a boundary.";
      continue;
    }
    ADEBUG << "  - The boundary is:";
    for (const auto &bd : lateral_bounds) {
      ADEBUG << std::get<0>(bd) << ": " << std::get<1>(bd) << ", "
             << std::get<2>(bd);
    }
    // 2. Call optimizer to generate smooth path.
    InitSolver();
    fem_qp_->SetVariableBounds(lateral_bounds);
    if (!fem_qp_->Optimize()) {
      can_side_pass[i] = false;
      ADEBUG << "  - Failed to optimize in SidePassPathDecider.";
      continue;
    }
    // 3. Update the path.
    double accumulated_s = adc_frenet_frame_point_.s();
    for (size_t j = 0; j < fem_qp_->x().size(); ++j) {
      common::FrenetFramePoint frenet_frame_point;
      ADEBUG << "FrenetFramePath: s = " << accumulated_s
             << ", l = " << fem_qp_->x()[j]
             << ", dl = " << fem_qp_->x_derivative()[j]
             << ", ddl = " << fem_qp_->x_second_order_derivative()[j];
      if (accumulated_s >= reference_line_info->reference_line().Length()) {
        break;
      }
      frenet_frame_point.set_s(accumulated_s);
      frenet_frame_point.set_l(fem_qp_->x()[j]);
      frenet_frame_point.set_dl(fem_qp_->x_derivative()[j]);
      frenet_frame_point.set_ddl(fem_qp_->x_second_order_derivative()[j]);
      frenet_frame_paths[i].push_back(std::move(frenet_frame_point));
      accumulated_s += delta_s_;
    }
    TrimGeneratedPath(&frenet_frame_paths[i]);
  }

  ADEBUG << "\n";
  // Decide a direction to side-pass.
  if (!DecideSidePassDirection(can_side_pass, frenet_frame_paths[0].size(),
                               frenet_frame_paths[1].size())) {
    ADEBUG << "Unable to generate path in either direction.";
    return false;
  }
  if (decided_direction_ == SidePassDirection::LEFT) {
    ADEBUG << "Decided to side-pass from LEFT.\n";
  } else {
    ADEBUG << "Decided to side-pass from RIGHT.\n";
  }
  auto path_data = reference_line_info->mutable_path_data();
  if (path_data == nullptr) {
    return false;
  }
  path_data->SetReferenceLine(&reference_line_info->reference_line());
  if (decided_direction_ == SidePassDirection::LEFT) {
    path_data->SetFrenetPath(FrenetFramePath(frenet_frame_paths[0]));
  } else {
    path_data->SetFrenetPath(FrenetFramePath(frenet_frame_paths[1]));
  }
  RecordDebugInfo(reference_line_info);
  return true;
}

std::vector<std::tuple<double, double, double>>
SidePassPathDecider::GetPathBoundaries(
    const TrajectoryPoint &planning_start_point,
    const SLBoundary &adc_sl_boundary, const ReferenceLine &reference_line,
    const IndexedList<std::string, Obstacle> &indexed_obstacles,
    bool *fail_to_find_boundary) {
  std::vector<std::tuple<double, double, double>> lateral_bounds;

  constexpr double kLargeBoundary = 10.0;
  std::unordered_map<std::string, SidePassDirection> obs_id_to_side_pass_dir;
  for (double curr_s = adc_frenet_frame_point_.s();
       curr_s < std::min(adc_frenet_frame_point_.s() + total_path_length_,
                         reference_line.Length());
       curr_s +=
       Decider::config_.side_pass_path_decider_config().path_resolution()) {
    std::tuple<double, double, double> lateral_bound = std::make_tuple(
        curr_s - adc_frenet_frame_point_.s(), -kLargeBoundary, kLargeBoundary);
    ADEBUG << "At curr_s = " << curr_s - adc_frenet_frame_point_.s();

    // Initialize the lateral bound with current lane's width.
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &lane_left_width,
                                     &lane_right_width)) {
      AERROR << "Fail to get lane width at s = " << curr_s;
      lateral_bounds.push_back(lateral_bound);
      continue;
    }
    const double curr_lane_width = lane_left_width + lane_right_width;
    const double adc_half_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    std::get<1>(lateral_bound) =
        -lane_right_width + adc_half_width + FLAGS_side_pass_road_buffer;
    std::get<2>(lateral_bound) =
        lane_left_width - adc_half_width - FLAGS_side_pass_road_buffer;
    ADEBUG << "lateral_bound initialized: " << std::get<1>(lateral_bound)
           << ", " << std::get<2>(lateral_bound) << ".";

    // Update the lateral bound based on the road info and side-pass direction.
    hdmap::Lane curr_lane;
    hdmap::LaneInfoConstPtr lane_info_ptr;
    if (!FLAGS_side_pass_use_actual_laneinfo_for_path_generation ||
        !GetLaneInfoFromPoint(
            reference_line.GetReferencePoint(curr_s).x(),
            reference_line.GetReferencePoint(curr_s).y(),
            0.0,
            reference_line.GetReferencePoint(curr_s).heading(),
            &lane_info_ptr)) {
      ADEBUG << "Cannot find the true current lane; therefore, use the "
                "planning starting point's current lane as a substitute.";
      curr_lane = curr_lane_;
    } else {
      curr_lane = lane_info_ptr->lane();
    }
    ADEBUG << "Current lane's ID: " << curr_lane.id().id();
    ADEBUG << "Current lane's width = " << curr_lane_width;
    ADEBUG << "Number of left lanes: "
           << curr_lane.left_neighbor_forward_lane_id_size() +
                  curr_lane.left_neighbor_reverse_lane_id_size()
           << ". Number of right lanes: "
           << curr_lane.right_neighbor_forward_lane_id_size() +
                  curr_lane.right_neighbor_reverse_lane_id_size();
    if (decided_direction_ == SidePassDirection::LEFT &&
        (curr_lane.left_neighbor_forward_lane_id_size() > 0 ||
         curr_lane.left_neighbor_reverse_lane_id_size() > 0)) {
      ADEBUG << "Expanding the upper limit (left).";
      double adjacent_lane_width = 0.0;
      hdmap::LaneInfoConstPtr adjacent_lane;
      // Get the LaneInfo of the left lane.
      if (curr_lane.left_neighbor_forward_lane_id_size() > 0) {
        adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
            curr_lane.left_neighbor_forward_lane_id(0));
      } else if (curr_lane.left_neighbor_reverse_lane_id_size() > 0) {
        adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
            curr_lane.left_neighbor_reverse_lane_id(0));
      }
      // Get the XY point of the curr_s.
      common::math::Vec2d xy_curr_s;
      common::SLPoint sl_curr_s;
      sl_curr_s.set_s(curr_s);
      sl_curr_s.set_l(0.0);
      reference_line.SLToXY(sl_curr_s, &xy_curr_s);
      // Get the projection of the XY point onto the left lane,
      // and use that s to get the left lane's width.
      double adjacent_lane_s = 0.0;
      double adjacent_lane_l = 0.0;
      if (!adjacent_lane->GetProjection(xy_curr_s, &adjacent_lane_s,
                                        &adjacent_lane_l)) {
        ADEBUG << "Unable to get the left lane's width due to "
                  "failure in getting projection.";
        ADEBUG << "Therefore, treat the current lane's width as"
                  "the left lane's width.";
        adjacent_lane_width = curr_lane_width;
      } else {
        adjacent_lane_width = adjacent_lane->GetWidth(adjacent_lane_s);
      }
      ADEBUG << "Upper limit expanded by " << adjacent_lane_width;
      // Update the lateral_bound accordingly.
      std::get<2>(lateral_bound) += adjacent_lane_width;
      std::get<2>(lateral_bound) -= FLAGS_side_pass_vehicle_buffer;
    } else if (decided_direction_ == SidePassDirection::RIGHT &&
               (curr_lane.right_neighbor_forward_lane_id_size() > 0 ||
                curr_lane.right_neighbor_reverse_lane_id_size() > 0)) {
      ADEBUG << "Expanding the lower limit (right).";
      double adjacent_lane_width = 0.0;
      hdmap::LaneInfoConstPtr adjacent_lane;
      // Get the LaneInfo of the right lane.
      if (curr_lane.right_neighbor_forward_lane_id_size() > 0) {
        adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
            curr_lane.right_neighbor_forward_lane_id(0));
      } else if (curr_lane.right_neighbor_reverse_lane_id_size() > 0) {
        adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
            curr_lane.right_neighbor_reverse_lane_id(0));
      }
      // Get the XY point of the curr_s.
      common::math::Vec2d xy_curr_s;
      common::SLPoint sl_curr_s;
      sl_curr_s.set_s(curr_s);
      sl_curr_s.set_l(0.0);
      reference_line.SLToXY(sl_curr_s, &xy_curr_s);
      // Get the projection of the XY point onto the left lane,
      // and use that s to get the left lane's width.
      double adjacent_lane_s = 0.0;
      double adjacent_lane_l = 0.0;
      if (!adjacent_lane->GetProjection(xy_curr_s, &adjacent_lane_s,
                                        &adjacent_lane_l)) {
        ADEBUG << "Unable to get the right lane's width due to "
                  "failure in getting projection.";
        ADEBUG << "Therefore, treat the current lane's width as"
                  "the right lane's width.";
        adjacent_lane_width = curr_lane_width;
      } else {
        adjacent_lane_width = adjacent_lane->GetWidth(adjacent_lane_s);
      }
      ADEBUG << "Lower limit expanded by " << adjacent_lane_width;
      std::get<1>(lateral_bound) -= adjacent_lane_width;
      std::get<1>(lateral_bound) += FLAGS_side_pass_vehicle_buffer;
    }
    ADEBUG << "lateral_bound updated based on direction: "
           << std::get<1>(lateral_bound) << ", " << std::get<2>(lateral_bound)
           << ".";

    // Update lateral_bound based on obstacles:
    for (const auto *obstacle : indexed_obstacles.Items()) {
      if (obstacle->IsVirtual() || !obstacle->IsStatic()) {
        continue;
      }
      const auto obs_sl = obstacle->PerceptionSLBoundary();
      // ADEBUG << obs_sl.ShortDebugString();
      // ADEBUG << "Offset = " << adc_frenet_frame_point_.s();
      // not overlap with obstacle
      if (curr_s < obs_sl.start_s() - FLAGS_side_pass_obstacle_s_buffer ||
          curr_s > obs_sl.end_s() + FLAGS_side_pass_obstacle_s_buffer) {
        continue;
      }
      // not within lateral range
      if (obs_sl.start_l() > std::get<2>(lateral_bound) +
                                 FLAGS_side_pass_obstacle_l_buffer +
                                 adc_half_width ||
          obs_sl.end_l() < std::get<1>(lateral_bound) -
                               FLAGS_side_pass_obstacle_l_buffer -
                               adc_half_width) {
        ADEBUG << "Obstacle not considered.";
        continue;
      }
      ADEBUG << "Obstacle within consideration: "
             << " start_s = " << obs_sl.start_s() - adc_frenet_frame_point_.s()
             << " end_s = " << obs_sl.end_s() - adc_frenet_frame_point_.s()
             << " start_l = " << obs_sl.start_l()
             << " end_l = " << obs_sl.end_l();
      ADEBUG << "ADC width = " << 2.0 * adc_half_width;
      *fail_to_find_boundary = false;

      SidePassDirection side_pass_direction = SidePassDirection::LEFT;
      if (obs_id_to_side_pass_dir.count(obstacle->Id()) == 0) {
        // We haven't decided the side-pass direction for this obstacle yet.
        if (std::get<2>(lateral_bound) - obs_sl.end_l() >
            obs_sl.start_l() - std::get<1>(lateral_bound)) {
          obs_id_to_side_pass_dir[obstacle->Id()] = SidePassDirection::LEFT;
        } else {
          obs_id_to_side_pass_dir[obstacle->Id()] = SidePassDirection::RIGHT;
        }
      }
      side_pass_direction = obs_id_to_side_pass_dir[obstacle->Id()];

      if (side_pass_direction == SidePassDirection::LEFT) {
        const double lower_bound = FLAGS_static_decision_nudge_l_buffer +
                                   obs_sl.end_l() + adc_half_width +
                                   FLAGS_side_pass_obstacle_l_buffer;
        ADEBUG << "Should pass from left. Lower bound = " << lower_bound;
        if (std::get<2>(lateral_bound) -
                FLAGS_side_pass_extra_road_buffer_during_turning >=
            lower_bound) {
          ADEBUG << "Reseting the boundaries for left-side-pass.";
          std::get<1>(lateral_bound) = lower_bound;
          std::get<2>(lateral_bound) -=
              FLAGS_side_pass_extra_road_buffer_during_turning;
        } else {
          *fail_to_find_boundary = true;
          break;
        }
      } else {
        const double upper_bound = -FLAGS_static_decision_nudge_l_buffer +
                                   obs_sl.start_l() - adc_half_width -
                                   FLAGS_side_pass_obstacle_l_buffer;
        ADEBUG << "Should pass from right. Upper bound = " << upper_bound;
        if (std::get<1>(lateral_bound) +
                FLAGS_side_pass_extra_road_buffer_during_turning <=
            upper_bound) {
          ADEBUG << "Reseting the boundaries for right-side-pass.";
          std::get<2>(lateral_bound) = upper_bound;
          std::get<1>(lateral_bound) +=
              FLAGS_side_pass_extra_road_buffer_during_turning;
        } else {
          *fail_to_find_boundary = true;
          break;
        }
      }
    }

    if (*fail_to_find_boundary) {
      ADEBUG << "Failed to get a feasible boundary.";
      return lateral_bounds;
    }

    ADEBUG << "lateral_bound updated based on obstacles: "
           << std::get<1>(lateral_bound) << ", " << std::get<2>(lateral_bound)
           << ".";
    ADEBUG << "\n";
    lateral_bounds.push_back(lateral_bound);
  }
  return lateral_bounds;
}

bool SidePassPathDecider::TrimGeneratedPath(
    std::vector<common::FrenetFramePoint> *ptr_frenet_frame_path) {
  // Sanity checks.
  if (ptr_frenet_frame_path->empty()) {
    return false;
  }
  if (std::fabs(ptr_frenet_frame_path->back().l()) >
      FLAGS_side_pass_off_road_center_threshold) {
    return false;
  }

  // If haven't departed at all, don't trim.
  // TODO(all): make trimming dependent on the obstacle location.
  bool ever_departed_reference_line = false;
  for (size_t k = 0; k < ptr_frenet_frame_path->size(); ++k) {
    if (std::fabs((*ptr_frenet_frame_path)[k].l()) >
        FLAGS_side_pass_off_road_center_threshold) {
      ever_departed_reference_line = true;
      break;
    }
  }
  if (!ever_departed_reference_line) {
    return false;
  }

  // Sliding window algorithm.
  int i = static_cast<int32_t>(ptr_frenet_frame_path->size()) - 1;
  int j = static_cast<int32_t>(ptr_frenet_frame_path->size()) - 1;
  // 1. Move j so that it is FLAGS_side_pass_trim_watch_window ahead of i.
  while (j >= 0) {
    if ((*ptr_frenet_frame_path)[i].s() - (*ptr_frenet_frame_path)[j].s() >
        FLAGS_side_pass_trim_watch_window) {
      break;
    }
    if (std::fabs((*ptr_frenet_frame_path)[j].l()) >
        FLAGS_side_pass_off_road_center_threshold) {
      return false;
    }
    j--;
  }
  if (j < 0) {
    return false;
  }
  // 2. Slide the j-i window backward until the point where side-pass finishes.
  //    Trim the tailing path points.
  while (j >= 0) {
    if (std::fabs((*ptr_frenet_frame_path)[j].l()) >
        FLAGS_side_pass_off_road_center_threshold) {
      break;
    }
    i--;
    j--;
    ptr_frenet_frame_path->pop_back();
  }
  return true;
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
    if (obs_start_l > lane_left_width || obs_end_l < -lane_right_width) {
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

void SidePassPathDecider::RecordDebugInfo(
    ReferenceLineInfo *const reference_line_info) {
  const auto &path_points = reference_line_info->path_data().discretized_path();
  auto *ptr_optimized_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

}  // namespace planning
}  // namespace apollo
