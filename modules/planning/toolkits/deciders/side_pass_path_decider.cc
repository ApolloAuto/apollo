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
using apollo::hdmap::PathOverlap;
using apollo::common::util::MakePointENU;
using apollo::hdmap::HDMapUtil;

constexpr double kRoadBuffer = 0.0;
constexpr double kObstacleLBuffer = 0.1;
constexpr double kObstacleSBuffer = 1.0;
constexpr double kSidePassPathLength = 50.0;

SidePassPathDecider::SidePassPathDecider(const TaskConfig &config)
    : Decider(config) {}

void SidePassPathDecider::InitSolver() {
  const TaskConfig &config = Decider::config_;
  fem_qp_.reset(new Fem1dExpandedJerkQpProblem());
  delta_s_ = config.side_pass_path_decider_config().path_resolution();
  const int n = static_cast<int>(
      config.side_pass_path_decider_config().total_path_length() / delta_s_);
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
  CHECK(fem_qp_->Init(n, l_init, delta_s_, w,
                      config.side_pass_path_decider_config().max_dddl()));
}

Status SidePassPathDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  adc_planning_start_point_ = frame->PlanningStartPoint();
  adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());
  InitSolver();

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

bool SidePassPathDecider::BuildSidePathDecision(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  hdmap::LaneInfoConstPtr lane;
  double s = 0.0;
  double l = 0.0;
  if (HDMapUtil::BaseMapPtr()->GetNearestLaneWithHeading(
          common::util::MakePointENU(
              adc_planning_start_point_.path_point().x(),
              adc_planning_start_point_.path_point().y(),
              adc_planning_start_point_.path_point().z()),
          1.0, adc_planning_start_point_.path_point().theta(), M_PI / 3.0,
          &lane, &s, &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << adc_planning_start_point_.DebugString()
           << ", heading:" << adc_planning_start_point_.path_point().theta();
    return false;
  }

  curr_lane_ = lane->lane();
  ADEBUG << curr_lane_.ShortDebugString();

  if (curr_lane_.left_neighbor_forward_lane_id_size() > 0) {
    decided_direction_ = SidePassDirection::LEFT;
  } else if (curr_lane_.right_neighbor_forward_lane_id_size() > 0) {
    decided_direction_ = SidePassDirection::RIGHT;
  } else if (curr_lane_.left_neighbor_reverse_lane_id_size() > 0) {
    decided_direction_ = SidePassDirection::LEFT;
  } else if (curr_lane_.right_neighbor_reverse_lane_id_size() > 0) {
    decided_direction_ = SidePassDirection::RIGHT;
  } else {
    const double obs_start_s =
        nearest_obstacle_->PerceptionSLBoundary().start_s();
    const double obs_end_s = nearest_obstacle_->PerceptionSLBoundary().end_s();

    // Filter out those out-of-lane obstacles.
    double lane_left_width_at_start_s = 0.0;
    double lane_right_width_at_start_s = 0.0;
    reference_line_info->reference_line().GetLaneWidth(
        obs_start_s, &lane_left_width_at_start_s, &lane_right_width_at_start_s);

    double lane_left_width_at_end_s = 0.0;
    double lane_right_width_at_end_s = 0.0;
    reference_line_info->reference_line().GetLaneWidth(
        obs_end_s, &lane_left_width_at_end_s, &lane_right_width_at_end_s);

    double lane_left_width = std::min(std::abs(lane_left_width_at_start_s),
                                      std::abs(lane_left_width_at_end_s));
    double lane_right_width = std::min(std::abs(lane_right_width_at_start_s),
                                       std::abs(lane_right_width_at_end_s));

    const double obs_start_l =
        nearest_obstacle_->PerceptionSLBoundary().start_l();
    const double obs_end_l = nearest_obstacle_->PerceptionSLBoundary().end_l();

    const double left_space = lane_left_width - obs_end_l;
    const double right_space = lane_right_width + obs_start_l;

    const double adc_half_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

    if (left_space > adc_half_width + kRoadBuffer) {
      decided_direction_ = SidePassDirection::LEFT;
    } else if (right_space > adc_half_width + kRoadBuffer) {
      decided_direction_ = SidePassDirection::RIGHT;
    } else {
      AERROR << "Fail to find side pass direction.";
      return false;
    }
  }

  if (decided_direction_ == SidePassDirection::LEFT) {
    ADEBUG << "Decided to side-pass from LEFT.";
  } else {
    ADEBUG << "Decided to side-pass from RIGHT.";
  }

  return true;
}

bool SidePassPathDecider::GeneratePath(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Decide whether to side-pass from left or right.
  if (!BuildSidePathDecision(frame, reference_line_info)) {
    AERROR << "Failed to decide on a side-pass direction.";
    return false;
  }

  bool fail_to_find_boundary = false;
  auto lateral_bounds = GetPathBoundaries(
      frame->PlanningStartPoint(), reference_line_info->AdcSlBoundary(),
      reference_line_info->reference_line(),
      reference_line_info->path_decision()->obstacles(),
      &fail_to_find_boundary);
  if (fail_to_find_boundary) {
    return false;
  }

  for (const auto &bd : lateral_bounds) {
    ADEBUG << std::get<0>(bd) << ": " << std::get<1>(bd) << ", "
           << std::get<2>(bd);
  }

  // Call optimizer to generate smooth path.
  fem_qp_->SetVariableBounds(lateral_bounds);
  if (!fem_qp_->Optimize()) {
    AERROR << "Fail to optimize in SidePassPathDecider.";
    return false;
  }

  // TODO(All): put optimized results into ReferenceLineInfo.
  // Update Reference_Line_Info with this newly generated path.
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = adc_frenet_frame_point_.s();
  for (size_t i = 0; i < fem_qp_->x().size(); ++i) {
    common::FrenetFramePoint frenet_frame_point;
    ADEBUG << "FrenetFramePath: s = " << accumulated_s
           << ", l = " << fem_qp_->x()[i]
           << ", dl = " << fem_qp_->x_derivative()[i]
           << ", ddl = " << fem_qp_->x_second_order_derivative()[i];
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
  if (path_data == nullptr) {
    return false;
  }
  path_data->SetReferenceLine(&reference_line_info->reference_line());
  path_data->SetFrenetPath(FrenetFramePath(frenet_frame_path));

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
       curr_s < std::min(adc_frenet_frame_point_.s() + kSidePassPathLength,
                         reference_line.Length());
       curr_s +=
       Decider::config_.side_pass_path_decider_config().path_resolution()) {
    std::tuple<double, double, double> lateral_bound = std::make_tuple(
        curr_s - adc_frenet_frame_point_.s(), -kLargeBoundary, kLargeBoundary);

    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &lane_left_width,
                                     &lane_right_width)) {
      AERROR << "Fail to get lane width at s = " << curr_s;
      lateral_bounds.push_back(lateral_bound);
      continue;
    }

    std::get<1>(lateral_bound) = -lane_right_width + kRoadBuffer;
    std::get<2>(lateral_bound) = lane_left_width - kRoadBuffer;

    const double lane_width = lane_left_width + lane_right_width;
    const double adc_half_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

    ADEBUG << "Number of left lanes: "
           << curr_lane_.left_neighbor_forward_lane_id_size() +
              curr_lane_.left_neighbor_reverse_lane_id_size();
    ADEBUG << "Number of right lanes: "
           << curr_lane_.right_neighbor_forward_lane_id_size() +
              curr_lane_.right_neighbor_reverse_lane_id_size();
    if (decided_direction_ == SidePassDirection::LEFT &&
        (curr_lane_.left_neighbor_forward_lane_id_size() > 0 ||
         curr_lane_.left_neighbor_reverse_lane_id_size() > 0)) {
      ADEBUG << "Expanding the upper limit (left).";
      std::get<2>(lateral_bound) =
          lane_left_width + lane_width - adc_half_width - kRoadBuffer;
    } else if (decided_direction_ == SidePassDirection::RIGHT &&
               (curr_lane_.right_neighbor_forward_lane_id_size() > 0 ||
                curr_lane_.right_neighbor_reverse_lane_id_size() > 0)) {
      ADEBUG << "Expanding the lower limit (right).";
      std::get<1>(lateral_bound) =
          -lane_right_width - lane_width + adc_half_width + kRoadBuffer;
    }

    for (const auto *obstacle : indexed_obstacles.Items()) {
      if (obstacle->IsVirtual()) {
        continue;
      }
      const auto obs_sl = obstacle->PerceptionSLBoundary();
      ADEBUG << obs_sl.ShortDebugString();

      // not overlap with obstacle
      if (curr_s < obs_sl.start_s() - kObstacleSBuffer ||
          curr_s > obs_sl.end_s() + kObstacleSBuffer) {
        continue;
      }
      // not within lateral range
      if (obs_sl.start_l() > std::get<2>(lateral_bound) ||
          obs_sl.end_l() < std::get<1>(lateral_bound)) {
        continue;
      }

      ADEBUG << "Obstacles that are considered is at: "
             << "curr_s = " << curr_s - adc_frenet_frame_point_.s()
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
        const double lower_bound =
            FLAGS_static_decision_nudge_l_buffer + obs_sl.end_l();
        ADEBUG << "Should pass from left. Lower bound = " << lower_bound;
        if (std::get<2>(lateral_bound) - lower_bound - 2.0 * adc_half_width -
                kRoadBuffer >=
            0.0) {
          ADEBUG << "Reseting the right boundary for left-side-pass.";
          std::get<1>(lateral_bound) =
              lower_bound + adc_half_width + kObstacleLBuffer;
        } else {
          *fail_to_find_boundary = true;
          break;
        }
      } else {
        const double upper_bound =
            -FLAGS_static_decision_nudge_l_buffer + obs_sl.start_l();
        ADEBUG << "Should pass from right. Upper bound = " << upper_bound;
        if (upper_bound - std::get<1>(lateral_bound) - 2.0 * adc_half_width -
                kRoadBuffer >=
            0.0) {
          ADEBUG << "Reseting the left boundary for right-side-pass.";
          std::get<2>(lateral_bound) =
              upper_bound - adc_half_width - kObstacleLBuffer;
        } else {
          *fail_to_find_boundary = true;
          break;
        }
      }
    }

    if (*fail_to_find_boundary) {
      return lateral_bounds;
    }

    ADEBUG << "obstacle bound: " << std::get<0>(lateral_bound) << ", "
           << std::get<1>(lateral_bound) << ", " << std::get<2>(lateral_bound);

    lateral_bounds.push_back(lateral_bound);
  }
  return lateral_bounds;
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
  const auto &path_points =
      reference_line_info->path_data().discretized_path().path_points();
  auto *ptr_optimized_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

}  // namespace planning
}  // namespace apollo
