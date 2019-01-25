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

#include "modules/planning/open_space/trajectory_partition/trajectory_partitioner.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::time::Clock;

TrajectoryPartitioner::TrajectoryPartitioner() {
  CHECK(common::util::GetProtoFromFile(FLAGS_planner_open_space_config_filename,
                                       &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  gear_shift_max_t_ = planner_open_space_config_.trajectory_partition_config()
                          .gear_shift_max_t();
  gear_shift_unit_t_ = planner_open_space_config_.trajectory_partition_config()
                           .gear_shift_unit_t();
  interpolated_pieces_num_ =
      planner_open_space_config_.trajectory_partition_config()
          .interpolated_pieces_num();
  initial_gear_check_horizon_ =
      planner_open_space_config_.trajectory_partition_config()
          .initial_gear_check_horizon();
  heading_searching_range_ =
      planner_open_space_config_.trajectory_partition_config()
          .heading_searching_range();
  gear_shift_period_duration_ =
      planner_open_space_config_.trajectory_partition_config()
          .gear_shift_period_duration();
}

Status TrajectoryPartitioner::TrajectoryPartition(
    const std::unique_ptr<PublishableTrajectory>& trajectory_to_be_partitioned,
    const Frame* frame, ADCTrajectory* const ptr_trajectory_pb) {
  // interpolate the zigzag trajectory
  std::vector<common::TrajectoryPoint> zigzag_trajectory;
  InterpolateTrajectory(trajectory_to_be_partitioned, &zigzag_trajectory);

  double distance_s = 0.0;
  apollo::planning_internal::Trajectories trajectory_partitioned;
  std::vector<apollo::canbus::Chassis::GearPosition> gear_positions;

  apollo::common::Trajectory* current_trajectory =
      trajectory_partitioned.add_trajectory();

  // set initial gear position for first ADCTrajectory depending on v
  // and check potential edge cases
  const double kepsilon = 1e-6;
  size_t horizon = zigzag_trajectory.size();
  size_t initial_horizon = std::min(horizon, initial_gear_check_horizon_);
  int direction_flag = 0;
  size_t i = 0;
  int j = 0;
  int init_direction = 0;
  while (i != initial_horizon) {
    if (zigzag_trajectory[j].v() > kepsilon) {
      ++i;
      ++j;
      direction_flag++;
      if (init_direction == 0) {
        init_direction++;
      }
    } else if (zigzag_trajectory[j].v() < -kepsilon) {
      ++i;
      ++j;
      direction_flag--;
      if (init_direction == 0) {
        init_direction--;
      }
    } else {
      ++j;
    }
  }
  if (direction_flag > 1) {
    gear_positions.push_back(canbus::Chassis::GEAR_DRIVE);
  } else if (direction_flag < -1) {
    gear_positions.push_back(canbus::Chassis::GEAR_REVERSE);
  } else {
    if (init_direction > 0) {
      ADEBUG << "initial speed oscillate too "
                "frequent around zero";
      gear_positions.push_back(canbus::Chassis::GEAR_DRIVE);
    } else if (init_direction < 0) {
      ADEBUG << "initial speed oscillate too "
                "frequent around zero";
      gear_positions.push_back(canbus::Chassis::GEAR_REVERSE);
    } else {
      ADEBUG << "Invalid trajectory start! initial speeds too small to decide "
                "gear";
      gear_positions.push_back(canbus::Chassis::GEAR_DRIVE);
    }
  }

  // partition trajectory points into each trajectory
  for (size_t i = 0; i < horizon; ++i) {
    // shift from GEAR_DRIVE to GEAR_REVERSE if v < 0
    // then add a new trajectory with GEAR_REVERSE
    if (zigzag_trajectory[i].v() < -kepsilon &&
        gear_positions.back() == canbus::Chassis::GEAR_DRIVE) {
      current_trajectory = trajectory_partitioned.add_trajectory();
      gear_positions.push_back(canbus::Chassis::GEAR_REVERSE);
      distance_s = 0.0;
    }
    // shift from GEAR_REVERSE to GEAR_DRIVE if v > 0
    // then add a new trajectory with GEAR_DRIVE
    if (zigzag_trajectory[i].v() > kepsilon &&
        gear_positions.back() == canbus::Chassis::GEAR_REVERSE) {
      current_trajectory = trajectory_partitioned.add_trajectory();
      gear_positions.push_back(canbus::Chassis::GEAR_DRIVE);
      distance_s = 0.0;
    }

    auto* point = current_trajectory->add_trajectory_point();
    point->set_relative_time(zigzag_trajectory[i].relative_time());
    point->mutable_path_point()->set_x(zigzag_trajectory[i].path_point().x());
    point->mutable_path_point()->set_y(zigzag_trajectory[i].path_point().y());
    point->mutable_path_point()->set_theta(
        zigzag_trajectory[i].path_point().theta());
    if (i > 0) {
      distance_s +=
          (gear_positions.back() == canbus::Chassis::GEAR_REVERSE ? -1.0
                                                                  : 1.0) *
          std::sqrt((zigzag_trajectory[i].path_point().x() -
                     zigzag_trajectory[i - 1].path_point().x()) *
                        (zigzag_trajectory[i].path_point().x() -
                         zigzag_trajectory[i - 1].path_point().x()) +
                    (zigzag_trajectory[i].path_point().y() -
                     zigzag_trajectory[i - 1].path_point().y()) *
                        (zigzag_trajectory[i].path_point().y() -
                         zigzag_trajectory[i - 1].path_point().y()));
    }
    point->mutable_path_point()->set_s(distance_s);

    point->set_v(zigzag_trajectory[i].v());
    const auto& vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();
    point->mutable_path_point()->set_kappa(
        std::tan(zigzag_trajectory[i].steer()) /
        vehicle_config.vehicle_param().wheel_base());
    point->set_a(zigzag_trajectory[i].a());
  }

  // Choose the one to follow based on the closest partitioned trajectory
  size_t trajectories_size = trajectory_partitioned.trajectory_size();
  size_t current_trajectory_index = 0;
  int closest_trajectory_point_index = 0;
  constexpr double kepsilon_to_destination = 1e-6;
  bool flag_change_to_next = false;
  // Could have a big error in vehicle state in single thread mode As the
  // vehicle state is only updated at the every beginning at RunOnce()
  VehicleState vehicle_state = frame->vehicle_state();

  auto comp = [](const std::pair<std::pair<size_t, int>, double>& left,
                 const std::pair<std::pair<size_t, int>, double>& right) {
    return left.second >= right.second;
  };
  std::priority_queue<std::pair<std::pair<size_t, int>, double>,
                      std::vector<std::pair<std::pair<size_t, int>, double>>,
                      decltype(comp)>
      closest_points(comp);

  for (size_t i = 0; i < trajectories_size; ++i) {
    double min_distance = std::numeric_limits<double>::max();
    const apollo::common::Trajectory trajectory =
        trajectory_partitioned.trajectory(static_cast<int>(i));
    int trajectory_size = trajectory.trajectory_point_size();

    const apollo::common::TrajectoryPoint trajectory_end_point =
        trajectory.trajectory_point(trajectory_size - 1);
    const apollo::common::PathPoint path_end_point =
        trajectory_end_point.path_point();

    double distance_to_trajs_end =
        (path_end_point.x() - vehicle_state.x()) *
            (path_end_point.x() - vehicle_state.x()) +
        (path_end_point.y() - vehicle_state.y()) *
            (path_end_point.y() - vehicle_state.y());

    double traj_point_moving_direction = path_end_point.theta();
    if (gear_positions[i] == canbus::Chassis::GEAR_REVERSE) {
      traj_point_moving_direction =
          common::math::NormalizeAngle(traj_point_moving_direction + M_PI);
    }
    double vehicle_moving_direction = vehicle_state.heading();
    if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
      vehicle_moving_direction =
          common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
    }

    if (distance_to_trajs_end <= kepsilon_to_destination &&
        std::abs(traj_point_moving_direction - vehicle_moving_direction) <
            heading_searching_range_) {
      if (i + 1 >= trajectories_size) {
        current_trajectory_index = trajectories_size - 1;
        closest_trajectory_point_index = trajectory_size - 1;
      } else {
        current_trajectory_index = i + 1;
        closest_trajectory_point_index = 0;
      }
      flag_change_to_next = true;
      break;
    }

    for (int j = 0; j < trajectory_size; ++j) {
      const apollo::common::TrajectoryPoint trajectory_point =
          trajectory.trajectory_point(j);
      const apollo::common::PathPoint path_point =
          trajectory_point.path_point();
      double distance = (path_point.x() - vehicle_state.x()) *
                            (path_point.x() - vehicle_state.x()) +
                        (path_point.y() - vehicle_state.y()) *
                            (path_point.y() - vehicle_state.y());
      if (distance < min_distance) {
        min_distance = distance;
        closest_trajectory_point_index = j;
      }
    }
    closest_points.push(std::make_pair(
        std::make_pair(i, closest_trajectory_point_index), min_distance));
  }

  if (!flag_change_to_next) {
    while (!closest_points.empty()) {
      auto closest_point = closest_points.top();
      closest_points.pop();
      double traj_point_moving_direction =
          trajectory_partitioned
              .trajectory(static_cast<int>(closest_point.first.first))
              .trajectory_point(closest_point.first.second)
              .path_point()
              .theta();
      if (gear_positions[closest_point.first.first] ==
          canbus::Chassis::GEAR_REVERSE) {
        traj_point_moving_direction =
            common::math::NormalizeAngle(traj_point_moving_direction + M_PI);
      }
      double vehicle_moving_direction = vehicle_state.heading();
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }
      if (std::abs(traj_point_moving_direction - vehicle_moving_direction) <
          heading_searching_range_) {
        current_trajectory_index = closest_point.first.first;
        closest_trajectory_point_index = closest_point.first.second;
        break;
      }
    }
  }

  if (FLAGS_use_gear_shift_trajectory) {
    if (InsertGearShiftTrajectory(flag_change_to_next, current_trajectory_index,
                                  gear_positions, frame, ptr_trajectory_pb)) {
      return Status::OK();
    }
  }

  SetTrajectoryPb(trajectory_partitioned, gear_positions,
                  current_trajectory_index, closest_trajectory_point_index,
                  ptr_trajectory_pb);
  return Status::OK();
}

void TrajectoryPartitioner::InterpolateTrajectory(
    const std::unique_ptr<PublishableTrajectory>& trajectory_to_be_partitioned,
    std::vector<common::TrajectoryPoint>* zigzag_trajectory) {
  size_t trajectory_to_be_partitioned_intervals_num =
      trajectory_to_be_partitioned->size() - 1;
  size_t interpolated_points_num = interpolated_pieces_num_ - 1;
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    double relative_time_interval =
        (trajectory_to_be_partitioned->at(i + 1).relative_time() -
         trajectory_to_be_partitioned->at(i).relative_time()) /
        static_cast<double>(interpolated_pieces_num_);
    zigzag_trajectory->push_back(trajectory_to_be_partitioned->at(i));
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          trajectory_to_be_partitioned->at(i).relative_time() +
          (static_cast<double>(j) + 1) * relative_time_interval;
      zigzag_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              trajectory_to_be_partitioned->at(i),
              trajectory_to_be_partitioned->at(i + 1), relative_time));
    }
  }

  zigzag_trajectory->push_back(trajectory_to_be_partitioned->back());
}

bool TrajectoryPartitioner::InsertGearShiftTrajectory(
    const bool& flag_change_to_next, const size_t& current_trajectory_index,
    const std::vector<apollo::canbus::Chassis::GearPosition>& gear_positions,
    const Frame* frame, ADCTrajectory* const ptr_trajectory_pb) {
  if (flag_change_to_next || !gear_shift_period_finished_) {
    gear_shift_period_finished_ = false;
    if (gear_shift_period_started_) {
      gear_shift_start_time_ = Clock::NowInSeconds();
      gear_shift_position_ = gear_positions[current_trajectory_index];
      gear_shift_period_started_ = false;
    }
    if (gear_shift_period_time_ > gear_shift_period_duration_) {
      gear_shift_period_finished_ = true;
      gear_shift_period_started_ = true;
    } else {
      GenerateGearShiftTrajectory(gear_shift_position_, frame,
                                  ptr_trajectory_pb);
      gear_shift_period_time_ = Clock::NowInSeconds() - gear_shift_start_time_;
      return true;
    }
  }
  return false;
}

void TrajectoryPartitioner::GenerateGearShiftTrajectory(
    const Chassis::GearPosition& gear_position, const Frame* frame,
    ADCTrajectory* trajectory_pb) {
  trajectory_pb->clear_trajectory_point();

  apollo::common::TrajectoryPoint tp;
  auto path_point = tp.mutable_path_point();
  path_point->set_x(frame->vehicle_state().x());
  path_point->set_y(frame->vehicle_state().y());
  path_point->set_theta(frame->vehicle_state().heading());
  path_point->set_kappa(frame->vehicle_state().kappa());
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < gear_shift_max_t_; t += gear_shift_unit_t_) {
    tp.set_relative_time(t);
    auto next_point = trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
  trajectory_pb->set_gear(gear_position);
}

void TrajectoryPartitioner::SetTrajectoryPb(
    const apollo::planning_internal::Trajectories& trajectory_partitioned,
    const std::vector<apollo::canbus::Chassis::GearPosition>& gear_positions,
    const size_t& current_trajectory_index,
    const int& closest_trajectory_point_index,
    ADCTrajectory* const ptr_trajectory_pb) {
  // reassign relative time and relative s to have the closest point as origin
  // point
  ptr_trajectory_pb->mutable_trajectory_point()->CopyFrom(
      trajectory_partitioned
          .trajectory(static_cast<int>(current_trajectory_index))
          .trajectory_point());
  double time_shift =
      ptr_trajectory_pb->trajectory_point(closest_trajectory_point_index)
          .relative_time();
  double s_shift =
      ptr_trajectory_pb->trajectory_point(closest_trajectory_point_index)
          .path_point()
          .s();
  int trajectory_size = ptr_trajectory_pb->trajectory_point_size();
  for (int i = 0; i < trajectory_size; ++i) {
    apollo::common::TrajectoryPoint* trajectory_point =
        ptr_trajectory_pb->mutable_trajectory_point(i);
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
  ptr_trajectory_pb->set_gear(gear_positions[current_trajectory_index]);
}

void TrajectoryPartitioner::Restart() {
  gear_shift_period_finished_ = true;
  gear_shift_period_started_ = true;
  gear_shift_period_time_ = 0.0;
  gear_shift_start_time_ = 0.0;
  gear_shift_position_ = canbus::Chassis::GEAR_DRIVE;
}
}  // namespace planning
}  // namespace apollo
