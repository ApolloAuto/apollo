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

/**
 * @file
 **/
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/tasks/optimizers/open_space_trajectory_partition/open_space_trajectory_partition.h"

#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using common::PathPoint;
using common::Status;
using common::TrajectoryPoint;
using common::math::Box2d;
using common::math::NormalizeAngle;
using common::math::Polygon2d;
using common::math::Vec2d;
using common::time::Clock;

OpenSpaceTrajectoryPartition::OpenSpaceTrajectoryPartition(
    const TaskConfig& config)
    : TrajectoryOptimizer(config) {
  open_space_trajectory_partition_config_ =
      config_.open_space_trajectory_partition_config();
}

void OpenSpaceTrajectoryPartition::Restart() {
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  current_gear_status->gear_switching_flag = false;
  current_gear_status->gear_shift_period_finished = true;
  current_gear_status->gear_shift_period_started = true;
  current_gear_status->gear_shift_period_time = 0.0;
  current_gear_status->gear_shift_start_time = 0.0;
  current_gear_status->gear_shift_position = canbus::Chassis::GEAR_DRIVE;
}

Status OpenSpaceTrajectoryPartition::Process() {
  const auto& open_space_info = frame_->open_space_info();
  auto open_space_info_ptr = frame_->mutable_open_space_info();
  const auto& trajectory = open_space_info.stitched_trajectory_result();

  auto interpolated_trajectory_result_ptr =
      open_space_info_ptr->mutable_interpolated_trajectory_result();

  InterpolateTrajectory(trajectory, interpolated_trajectory_result_ptr);

  auto paritioned_trajectories_ptr =
      open_space_info_ptr->mutable_paritioned_trajectories();
  double distance_s = 0.0;

  paritioned_trajectories_ptr->emplace_back();
  TrajGearPair* current_trajectory = &(paritioned_trajectories_ptr->back());
  // Set initial gear position for first ADCTrajectory depending on v
  // and check potential edge cases
  constexpr double kepsilon = 1e-8;
  size_t horizon = interpolated_trajectory_result_ptr->size();
  size_t initial_horizon = std::min(
      horizon, static_cast<size_t>(open_space_trajectory_partition_config_
                                       .initial_gear_check_horizon()));
  int direction_flag = 0;
  int init_direction = 0;
  for (size_t i = 0; i < initial_horizon; ++i) {
    if (interpolated_trajectory_result_ptr->at(i).v() > kepsilon) {
      direction_flag++;
      if (init_direction == 0) {
        init_direction++;
      }
    } else if (interpolated_trajectory_result_ptr->at(i).v() < -kepsilon) {
      direction_flag--;
      if (init_direction == 0) {
        init_direction--;
      }
    }
  }

  if (direction_flag > 1) {
    current_trajectory->second = canbus::Chassis::GEAR_DRIVE;
  } else if (direction_flag < -1) {
    current_trajectory->second = canbus::Chassis::GEAR_REVERSE;
  } else {
    if (init_direction > 0) {
      ADEBUG << "initial speed oscillate too frequent around zero";
      current_trajectory->second = canbus::Chassis::GEAR_DRIVE;
    } else if (init_direction < 0) {
      ADEBUG << "initial speed oscillate too frequent around zero";
      current_trajectory->second = canbus::Chassis::GEAR_REVERSE;
    } else {
      ADEBUG << "Invalid trajectory start! Speed values of initial points are "
                "too small to decide gear";
      current_trajectory->second = canbus::Chassis::GEAR_DRIVE;
    }
  }

  // Partition trajectory points into each trajectory
  for (size_t i = 0; i < horizon; ++i) {
    // shift from GEAR_DRIVE to GEAR_REVERSE if v < 0
    // then add a new trajectory with GEAR_REVERSE
    const TrajectoryPoint& trajectory_point_i =
        interpolated_trajectory_result_ptr->at(i);
    if (trajectory_point_i.v() < -kepsilon &&
        current_trajectory->second == canbus::Chassis::GEAR_DRIVE) {
      paritioned_trajectories_ptr->emplace_back();
      current_trajectory = &(paritioned_trajectories_ptr->back());
      current_trajectory->second = canbus::Chassis::GEAR_REVERSE;
      distance_s = 0.0;
    }
    // Shift from GEAR_REVERSE to GEAR_DRIVE if v > 0
    // then add a new trajectory with GEAR_DRIVE
    if (trajectory_point_i.v() > kepsilon &&
        current_trajectory->second == canbus::Chassis::GEAR_REVERSE) {
      paritioned_trajectories_ptr->emplace_back();
      current_trajectory = &(paritioned_trajectories_ptr->back());
      current_trajectory->second = canbus::Chassis::GEAR_DRIVE;
      distance_s = 0.0;
    }

    current_trajectory->first.emplace_back();
    TrajectoryPoint* point = &(current_trajectory->first.back());
    point->set_relative_time(trajectory_point_i.relative_time());
    point->mutable_path_point()->set_x(trajectory_point_i.path_point().x());
    point->mutable_path_point()->set_y(trajectory_point_i.path_point().y());
    point->mutable_path_point()->set_theta(
        trajectory_point_i.path_point().theta());
    if (i > 0) {
      const PathPoint& path_point_i = trajectory_point_i.path_point();
      const PathPoint& path_point_pre_i =
          interpolated_trajectory_result_ptr->at(i - 1).path_point();
      distance_s +=
          (current_trajectory->second == canbus::Chassis::GEAR_REVERSE ? -1.0
                                                                       : 1.0) *
          std::sqrt((path_point_i.x() - path_point_pre_i.x()) *
                        (path_point_i.x() - path_point_pre_i.x()) +
                    (path_point_i.y() - path_point_pre_i.y()) *
                        (path_point_i.y() - path_point_pre_i.y()));
    }
    point->mutable_path_point()->set_s(distance_s);

    point->set_v(trajectory_point_i.v());
    const auto& vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();
    point->mutable_path_point()->set_kappa(
        std::tan(trajectory_point_i.steer()) /
        vehicle_config.vehicle_param().wheel_base());
    point->set_a(trajectory_point_i.a());
  }

  // Choose the one to follow based on the closest partitioned trajectory
  size_t trajectories_size = paritioned_trajectories_ptr->size();
  size_t current_trajectory_index = 0;
  size_t current_trajectory_point_index = 0;
  bool flag_change_to_next = false;
  const double distance_search_range =
      open_space_trajectory_partition_config_.distance_search_range();
  const double distance_to_midpoint =
      open_space_trajectory_partition_config_.distance_to_midpoint();
  const double heading_search_range =
      open_space_trajectory_partition_config_.heading_search_range();
  const double heading_track_range =
      open_space_trajectory_partition_config_.heading_track_range();
  // Could have a big error in vehicle state in single thread mode As the
  // vehicle state is only updated at the every beginning at RunOnce()
  const common::VehicleState& vehicle_state = frame_->vehicle_state();

  auto pair_comp =
      [](const std::pair<std::pair<size_t, size_t>, double>& left,
         const std::pair<std::pair<size_t, size_t>, double>& right) {
        return left.second <= right.second;
      };
  auto comp = [](const std::pair<size_t, double>& left,
                 const std::pair<size_t, double>& right) {
    return left.second <= right.second;
  };

  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      decltype(pair_comp)>
      closest_point_on_trajs(pair_comp);

  // Build vehicle ego box for IoU tracking metric
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double ego_length = vehicle_config.vehicle_param().length();
  const double ego_width = vehicle_config.vehicle_param().width();
  const double ego_theta = vehicle_state.heading();
  const double ego_x = vehicle_state.x();
  const double ego_y = vehicle_state.y();
  Box2d ego_box({ego_x, ego_y}, ego_theta, ego_length, ego_width);
  const double shift_distance =
      ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
  Vec2d ego_shift_vec{shift_distance * std::cos(ego_theta),
                      shift_distance * std::sin(ego_theta)};
  ego_box.Shift(ego_shift_vec);
  const double vehicle_moving_direction =
      vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(vehicle_state.heading() + M_PI)
          : vehicle_state.heading();

  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = paritioned_trajectories_ptr->at(i).first;

    size_t trajectory_size = trajectory.size();

    // Check if have reached endpoint of trajectory
    const TrajectoryPoint& trajectory_end_point = trajectory.back();
    const PathPoint& path_end_point = trajectory_end_point.path_point();
    const double path_end_point_x = path_end_point.x();
    const double path_end_point_y = path_end_point.y();
    const double path_end_point_theta = path_end_point.theta();
    const double distance_to_trajs_end =
        std::sqrt((path_end_point_x - ego_x) * (path_end_point_x - ego_x) +
                  (path_end_point_y - ego_y) * (path_end_point_y - ego_y));
    const double traj_end_point_moving_direction =
        paritioned_trajectories_ptr->at(i).second ==
                canbus::Chassis::GEAR_REVERSE
            ? NormalizeAngle(path_end_point_theta + M_PI)
            : path_end_point_theta;

    const double heading_search_to_trajs_end = std::abs(NormalizeAngle(
        traj_end_point_moving_direction - vehicle_moving_direction));

    // If close to the end point, start on the next trajectory
    if (distance_to_trajs_end < distance_to_midpoint &&
        heading_search_to_trajs_end < heading_search_range) {
      // get vehicle box and path point box, compare with a threadhold in IOU
      Box2d path_end_point_box({path_end_point_x, path_end_point_y},
                               path_end_point_theta, ego_length, ego_width);
      Vec2d shift_vec{shift_distance * std::cos(path_end_point_theta),
                      shift_distance * std::sin(path_end_point_theta)};
      path_end_point_box.Shift(shift_vec);
      double end_point_iou_ratio =
          Polygon2d(ego_box).ComputeIoU(Polygon2d(path_end_point_box));

      if (end_point_iou_ratio >
          open_space_trajectory_partition_config_.vehicle_box_iou_threshold()) {
        if (i + 1 >= trajectories_size) {
          current_trajectory_index = trajectories_size - 1;
          current_trajectory_point_index = trajectory_size - 1;
        } else {
          current_trajectory_index = i + 1;
          current_trajectory_point_index = 0;
        }
        flag_change_to_next = true;
        break;
      }
    }

    // Choose the closest point to track
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, decltype(comp)>
        closest_point(comp);
    for (size_t j = 0; j < trajectory_size; ++j) {
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      const Vec2d tracking_vector(path_point_x - ego_x, path_point_y - ego_y);
      const double distance = tracking_vector.Length();
      const double tracking_direction = tracking_vector.Angle();
      const double traj_point_moving_direction =
          paritioned_trajectories_ptr->at(i).second ==
                  canbus::Chassis::GEAR_REVERSE
              ? NormalizeAngle(path_point_theta + M_PI)
              : path_point_theta;
      const double head_track_difference = std::abs(
          NormalizeAngle(tracking_direction - vehicle_moving_direction));
      const double heading_search_difference = std::abs(NormalizeAngle(
          traj_point_moving_direction - vehicle_moving_direction));

      if (distance < distance_search_range &&
          head_track_difference < heading_track_range &&
          heading_search_difference < heading_search_range) {
        // get vehicle box and path point box, compute IOU
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length, ego_width);
        Vec2d shift_vec{shift_distance * std::cos(path_point_theta),
                        shift_distance * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box).ComputeIoU(Polygon2d(path_point_box));
        closest_point.emplace(j, iou_ratio);
      }
    }

    if (!closest_point.empty()) {
      size_t closest_point_index = closest_point.top().first;
      double max_iou_ratio = closest_point.top().second;
      closest_point_on_trajs.emplace(std::make_pair(i, closest_point_index),
                                     max_iou_ratio);
    }
  }

  if (!flag_change_to_next) {
    if (closest_point_on_trajs.empty()) {
      std::string msg("Fail to find nearest trajectory point to follow");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    current_trajectory_index = closest_point_on_trajs.top().first.first;
    current_trajectory_point_index = closest_point_on_trajs.top().first.second;
  }

  auto chosen_paritioned_trajectory =
      open_space_info_ptr->mutable_chosen_paritioned_trajectory();

  const auto& paritioned_trajectories =
      open_space_info.paritioned_trajectories();

  if (FLAGS_use_gear_shift_trajectory) {
    if (InsertGearShiftTrajectory(flag_change_to_next, current_trajectory_index,
                                  paritioned_trajectories,
                                  chosen_paritioned_trajectory)) {
      return Status::OK();
    }
  }
  AdjustRelativeTimeAndS(paritioned_trajectories, current_trajectory_index,
                         current_trajectory_point_index,
                         chosen_paritioned_trajectory);
  return Status::OK();
}

void OpenSpaceTrajectoryPartition::InterpolateTrajectory(
    const DiscretizedTrajectory& trajectory,
    DiscretizedTrajectory* interpolated_trajectory) {
  interpolated_trajectory->clear();
  size_t interpolated_pieces_num =
      open_space_trajectory_partition_config_.interpolated_pieces_num();
  CHECK_GT(trajectory.size(), 0);
  CHECK_GT(interpolated_pieces_num, 0);
  size_t trajectory_to_be_partitioned_intervals_num = trajectory.size() - 1;
  size_t interpolated_points_num = interpolated_pieces_num - 1;
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    double relative_time_interval =
        (trajectory.at(i + 1).relative_time() -
         trajectory.at(i).relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    interpolated_trajectory->push_back(trajectory.at(i));
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          trajectory.at(i).relative_time() +
          (static_cast<double>(j) + 1) * relative_time_interval;
      interpolated_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              trajectory.at(i), trajectory.at(i + 1), relative_time));
    }
  }
  interpolated_trajectory->push_back(trajectory.back());
}

bool OpenSpaceTrajectoryPartition::InsertGearShiftTrajectory(
    const bool flag_change_to_next, const size_t current_trajectory_index,
    const std::vector<TrajGearPair>& paritioned_trajectories,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  const auto* last_frame = FrameHistory::Instance()->Latest();
  const auto& last_gear_status =
      last_frame->open_space_info().gear_switch_states();
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  *(current_gear_status) = last_gear_status;

  if (flag_change_to_next || !current_gear_status->gear_shift_period_finished) {
    current_gear_status->gear_shift_period_finished = false;
    if (current_gear_status->gear_shift_period_started) {
      current_gear_status->gear_shift_start_time = Clock::NowInSeconds();
      current_gear_status->gear_shift_position =
          paritioned_trajectories.at(current_trajectory_index).second;
      current_gear_status->gear_shift_period_started = false;
    }
    if (current_gear_status->gear_shift_period_time >
        open_space_trajectory_partition_config_.gear_shift_period_duration()) {
      current_gear_status->gear_shift_period_finished = true;
      current_gear_status->gear_shift_period_started = true;
    } else {
      GenerateGearShiftTrajectory(current_gear_status->gear_shift_position,
                                  gear_switch_idle_time_trajectory);
      current_gear_status->gear_shift_period_time =
          Clock::NowInSeconds() - current_gear_status->gear_shift_start_time;
      return true;
    }
  }

  return true;
}

void OpenSpaceTrajectoryPartition::GenerateGearShiftTrajectory(
    const canbus::Chassis::GearPosition& gear_position,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  gear_switch_idle_time_trajectory->first.clear();
  const double gear_shift_max_t =
      open_space_trajectory_partition_config_.gear_shift_max_t();
  const double gear_shift_unit_t =
      open_space_trajectory_partition_config_.gear_shift_unit_t();
  const double vehicle_x = frame_->vehicle_state().x();
  const double vehicle_y = frame_->vehicle_state().y();
  const double vehicle_heading = frame_->vehicle_state().heading();
  const double vehicle_kappa = frame_->vehicle_state().kappa();
  for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
    gear_switch_idle_time_trajectory->first.emplace_back();
    auto* trajectory_point = &(gear_switch_idle_time_trajectory->first.back());
    auto* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(vehicle_x);
    path_point->set_y(vehicle_y);
    path_point->set_theta(vehicle_heading);
    path_point->set_kappa(vehicle_kappa);
    path_point->set_s(0.0);
    trajectory_point->set_v(0.0);
    trajectory_point->set_a(0.0);
    trajectory_point->set_relative_time(t);
  }
  gear_switch_idle_time_trajectory->second = gear_position;
}

void OpenSpaceTrajectoryPartition::AdjustRelativeTimeAndS(
    const std::vector<TrajGearPair>& paritioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    TrajGearPair* current_paritioned_trajectory) {
  // Reassign relative time and relative s to have the closest point as origin
  // point
  *(current_paritioned_trajectory) =
      paritioned_trajectories.at(current_trajectory_index);
  auto trajectory = &(current_paritioned_trajectory->first);
  double time_shift =
      trajectory->at(closest_trajectory_point_index).relative_time();
  double s_shift =
      trajectory->at(closest_trajectory_point_index).path_point().s();
  size_t trajectory_size = trajectory->size();
  for (size_t i = 0; i < trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
}

}  // namespace planning
}  // namespace apollo
