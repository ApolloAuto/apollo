/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/planning/tasks/open_space_trajectory_post_process/open_space_trajectory_post_process.h"

#include <algorithm>
#include <memory>
#include <queue>

#include "absl/strings/str_cat.h"

#include "cyber/time/clock.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

bool OpenSpaceTrajectoryPostProcess::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  bool res = Task::LoadConfig<OpenSpaceTrajectoryPostProcessConfig>(&config_);
  heading_search_range_ = config_.heading_search_range();
  heading_track_range_ = config_.heading_track_range();
  distance_search_range_ = config_.distance_search_range();
  heading_offset_to_midpoint_ = config_.heading_offset_to_midpoint();
  lateral_offset_to_midpoint_ = config_.lateral_offset_to_midpoint();
  longitudinal_offset_to_midpoint_ = config_.longitudinal_offset_to_midpoint();
  vehicle_box_iou_threshold_to_midpoint_ =
      config_.vehicle_box_iou_threshold_to_midpoint();
  scale_destination_ = config_.scale_destination();
  stop_check_window_ = config_.stop_check_window();
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ego_length_ = vehicle_param_.length();
  ego_width_ = vehicle_param_.width();
  shift_distance_ = ego_length_ / 2.0 - vehicle_param_.back_edge_to_center();
  wheel_base_ = vehicle_param_.wheel_base();
  AINFO << config_.DebugString();
  return res;
}

Status OpenSpaceTrajectoryPostProcess::Process() {
  const auto& open_space_info = frame_->open_space_info();
  auto open_space_info_ptr = frame_->mutable_open_space_info();
  if (open_space_info_ptr->optimizer_trajectory_data().empty()) {
    AINFO << "optimize trajectory empty";
    return Status::OK();
  }

  const Frame* previous_frame = injector_->frame_history()->Latest();
  if (previous_frame != nullptr) {
    *(open_space_info_ptr->mutable_partitioned_trajectories()) =
        previous_frame->open_space_info().partitioned_trajectories();
    *(open_space_info_ptr->mutable_interpolated_trajectory_result()) =
        previous_frame->open_space_info().interpolated_trajectory_result();
  }

  if (open_space_info_ptr->partitioned_trajectories().empty()) {
    const auto& optimized_trajectory_result =
        open_space_info.optimizer_trajectory_data();

    auto* interpolated_trajectory_result_ptr =
        open_space_info_ptr->mutable_interpolated_trajectory_result();

    InterpolateTrajectory(optimized_trajectory_result,
                          interpolated_trajectory_result_ptr);

    auto* partitioned_trajectories =
        open_space_info_ptr->mutable_partitioned_trajectories();

    PartitionTrajectory(*interpolated_trajectory_result_ptr,
                        partitioned_trajectories);

    auto* open_space_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    open_space_status->set_position_init(true);
    auto* chosen_partitioned_trajectory =
        open_space_info_ptr->mutable_chosen_partitioned_trajectory();
    AdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(), 0, 0,
                           chosen_partitioned_trajectory);
    current_trajectory_index_ = 0;
    fail_search_fallback_ = false;
    last_index_ = -1;
    return Status::OK();
  }

  auto* partitioned_trajectories =
        open_space_info_ptr->mutable_partitioned_trajectories();
  // Choose the one to follow based on the closest partitioned trajectory
  size_t trajectories_size = partitioned_trajectories->size();
  // may get stop trajecotry;
  current_trajectory_index_ =
      std::min<size_t>(trajectories_size - 1, current_trajectory_index_);
  size_t current_trajectory_point_index = 0;
  bool flag_change_to_next = false;

  // Vehicle related information used to choose closest point
  UpdateVehicleInfo();
  AINFO << "current_trajectory_index_" << current_trajectory_index_;
  const auto& gear =
      partitioned_trajectories->at(current_trajectory_index_).second;
  const auto& cur_trajectory =
      partitioned_trajectories->at(current_trajectory_index_).first;
  size_t trajectory_size = cur_trajectory.size();
  CHECK_GT(trajectory_size, 0U);
  flag_change_to_next = CheckReachTrajectoryEnd(
      cur_trajectory, gear, trajectories_size, &current_trajectory_point_index);
  AINFO << "current_trajectory_index_" << current_trajectory_index_ << ","
        << current_trajectory_point_index << "time index" << last_index_
        << "flag_change_to_next" << flag_change_to_next;
  auto* chosen_partitioned_trajectory =
      open_space_info_ptr->mutable_chosen_partitioned_trajectory();

  // check if ego arrive destination
  if (current_trajectory_index_ == partitioned_trajectories->size() - 1
      && current_trajectory_point_index == cur_trajectory.size() - 1) {
    AINFO << "reach destination";
    frame_->mutable_open_space_info()->set_destination_reached(true);
    GenerateStopTrajectory(
        frame_->local_view().chassis->gear_location(),
        chosen_partitioned_trajectory);
    return Status::OK();
  }

  auto chosen_trajectory = &(chosen_partitioned_trajectory->first);
  AINFO << "Before InsertGearShiftTrajectory [" << chosen_trajectory->size();
  if (config_.use_gear_shift_trajectory()) {
    if (InsertGearShiftTrajectory(flag_change_to_next,
                                  current_trajectory_index_,
                                  open_space_info.partitioned_trajectories(),
                                  chosen_partitioned_trajectory) &&
        chosen_partitioned_trajectory->first.size() != 0) {
      chosen_trajectory = &(chosen_partitioned_trajectory->first);
      ADEBUG << "After InsertGearShiftTrajectory [" << chosen_trajectory->size()
             << "]";
      last_index_ = -1;
      return Status::OK();
    }
  }
  AINFO << "Before InsertStopTrajectory [" << chosen_trajectory->size();
  // Choose the closest point to track
  std::priority_queue<std::pair<size_t, double>,
                      std::vector<std::pair<size_t, double>>, comp_>
      closest_point;
  AINFO << "ego:" << std::fixed << ego_x_ << "," << ego_y_ << ","
        << vehicle_moving_direction_;
  for (size_t j = 0; j < trajectory_size; ++j) {
    const TrajectoryPoint& trajectory_point = cur_trajectory.at(j);
    const PathPoint& path_point = trajectory_point.path_point();
    const double path_point_x = path_point.x();
    const double path_point_y = path_point.y();
    const double path_point_theta = path_point.theta();
    const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
    const double distance = tracking_vector.Length();
    // const double tracking_direction = tracking_vector.Angle();
    const double traj_point_moving_direction =
        gear == canbus::Chassis::GEAR_REVERSE
            ? NormalizeAngle(path_point_theta + M_PI)
            : path_point_theta;
    // const double head_track_difference = std::abs(
    //     NormalizeAngle(tracking_direction - vehicle_moving_direction_));
    const double heading_search_difference = std::abs(NormalizeAngle(
        traj_point_moving_direction - vehicle_moving_direction_));
    // AINFO << "XY" << std::fixed << path_point_x << "," << path_point_y << ","
    //       << path_point_theta << "," << distance << ","
    //       << heading_search_difference << "ego heading"
    //       << vehicle_moving_direction_;
    if (distance < distance_search_range_ &&
        heading_search_difference < heading_search_range_) {
      // get vehicle box and path point box, compute IOU
      Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                           ego_length_, ego_width_);
      Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                      shift_distance_ * std::sin(path_point_theta)};
      path_point_box.Shift(shift_vec);
      double iou_ratio =
          Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
      AINFO << std::fixed << "get closetst point" << path_point_x << ","
            << path_point_y << " distance" << distance << "iou" << iou_ratio
            << "pt theta" << traj_point_moving_direction;
      closest_point.emplace(j, iou_ratio);
    }
  }

  AINFO << "closest_point size" << closest_point.size() << " stop_check_count_: " << stop_check_count_;
  static constexpr int stop_count_replan_threshold = 300;
  if (closest_point.empty() || fail_search_fallback_ || stop_check_count_ > stop_count_replan_threshold) {
    frame_->mutable_open_space_info()->
        mutable_optimizer_trajectory_data()->clear();
    frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()->clear();
    frame_->mutable_open_space_info()->
        mutable_interpolated_trajectory_result()->clear();
    frame_->mutable_open_space_info()->
        mutable_partitioned_trajectories()->clear();
    frame_->mutable_open_space_info()->
        mutable_chosen_partitioned_trajectory()->first.clear();
    const std::string msg =
        "Fail to find nearest trajectory point to follow stop to fallback "
        "replan";
    AERROR << msg;
    stop_check_count_ = 0;
    return Status::OK();
  }
  current_trajectory_point_index = closest_point.top().first;
  if (!flag_change_to_next) {
    double veh_rel_time;
    size_t time_match_index;
    double now_time = Clock::Instance()->NowInSeconds();
    auto& traj = partitioned_trajectories->at(current_trajectory_index_).first;
    if (last_index_ != -1) {
      veh_rel_time = traj[last_index_].relative_time() + now_time - last_time_;
      AINFO << std::fixed << now_time << "," << last_time_;
      time_match_index = traj.QueryLowerBoundPoint(veh_rel_time);
    } else {
      time_match_index = current_trajectory_point_index;
      last_index_ = time_match_index;
      last_time_ = now_time;
    }

    AINFO << "time_match_index" << time_match_index << "pos match index"
          << current_trajectory_point_index;
    AINFO << "TRAJ CLOSEST" << std::fixed
          << traj.at(current_trajectory_point_index).path_point().x() << ","
          << traj.at(current_trajectory_point_index).path_point().y();
    if (std::abs(traj[time_match_index].path_point().s() -
                 traj.at(current_trajectory_point_index).path_point().s()) <
        config_.speed_replan_distance()) {
      current_trajectory_point_index = time_match_index;
    } else {
      AINFO << "reset speed because matched point too far";
      last_index_ = current_trajectory_point_index;
      last_time_ = now_time;
    }
  } else {
    last_index_ = -1;
  }

  AINFO << "current_trajectory_point_index" << current_trajectory_point_index;
  AdjustRelativeTimeAndS(open_space_info.partitioned_trajectories(),
                         current_trajectory_index_,
                         current_trajectory_point_index,
                         chosen_partitioned_trajectory);
  return Status::OK();
}

void OpenSpaceTrajectoryPostProcess::InterpolateTrajectory(
    const DiscretizedTrajectory& optimized_trajectory_result,
    DiscretizedTrajectory* interpolated_trajectory) {
  if (FLAGS_use_iterative_anchoring_smoother) {
    *interpolated_trajectory = optimized_trajectory_result;
    return;
  }
  interpolated_trajectory->clear();
  size_t interpolated_pieces_num = config_.interpolated_pieces_num();
  CHECK_GT(optimized_trajectory_result.size(), 0U);
  CHECK_GT(interpolated_pieces_num, 0U);
  size_t trajectory_to_be_partitioned_intervals_num =
      optimized_trajectory_result.size() - 1;
  size_t interpolated_points_num = interpolated_pieces_num - 1;
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    double relative_time_interval =
        (optimized_trajectory_result.at(i + 1).relative_time() -
         optimized_trajectory_result.at(i).relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    interpolated_trajectory->push_back(optimized_trajectory_result.at(i));
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          optimized_trajectory_result.at(i).relative_time() +
          (static_cast<double>(j) + 1.0) * relative_time_interval;
      interpolated_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              optimized_trajectory_result.at(i),
              optimized_trajectory_result.at(i + 1), relative_time));
    }
  }
  interpolated_trajectory->push_back(optimized_trajectory_result.back());
}

void OpenSpaceTrajectoryPostProcess::UpdateVehicleInfo() {
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  ego_theta_ = vehicle_state.heading();
  ego_x_ = vehicle_state.x();
  ego_y_ = vehicle_state.y();
  ego_v_ = vehicle_state.linear_velocity();
  ego_gear_ = vehicle_state.gear();
  Box2d box({ego_x_, ego_y_}, ego_theta_, ego_length_, ego_width_);
  ego_box_ = std::move(box);
  Vec2d ego_shift_vec{shift_distance_ * std::cos(ego_theta_),
                      shift_distance_ * std::sin(ego_theta_)};
  ego_box_.Shift(ego_shift_vec);
  vehicle_moving_direction_ =
      vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(ego_theta_ + M_PI)
          : ego_theta_;
  AINFO << fabs(ego_v_) << "speed" << vehicle_param_.max_abs_speed_when_stopped() << " " << stop_check_count_;
  stop_check_count_ = fabs(ego_v_) < vehicle_param_.max_abs_speed_when_stopped() ?
                          stop_check_count_ + 1 : 0;
}

bool OpenSpaceTrajectoryPostProcess::EncodeTrajectory(
    const DiscretizedTrajectory& trajectory, std::string* const encoding) {
  if (trajectory.empty()) {
    AERROR << "Fail to encode trajectory because it is empty";
    return false;
  }
  static constexpr double encoding_origin_x = 58700.0;
  static constexpr double encoding_origin_y = 4141000.0;
  const auto& init_path_point = trajectory.front().path_point();
  const auto& last_path_point = trajectory.back().path_point();

  const int init_point_x =
      static_cast<int>((init_path_point.x() - encoding_origin_x) * 1000.0);
  const int init_point_y =
      static_cast<int>((init_path_point.y() - encoding_origin_y) * 1000.0);
  const int init_point_heading =
      static_cast<int>(init_path_point.theta() * 10000.0);
  const int last_point_x =
      static_cast<int>((last_path_point.x() - encoding_origin_x) * 1000.0);
  const int last_point_y =
      static_cast<int>((last_path_point.y() - encoding_origin_y) * 1000.0);
  const int last_point_heading =
      static_cast<int>(last_path_point.theta() * 10000.0);

  *encoding = absl::StrCat(
      // init point
      init_point_x, "_", init_point_y, "_", init_point_heading, "/",
      // last point
      last_point_x, "_", last_point_y, "_", last_point_heading);
  return true;
}

bool OpenSpaceTrajectoryPostProcess::CheckTrajTraversed(
    const std::string& trajectory_encoding_to_check) {
  const auto& open_space_status =
      injector_->planning_context()->planning_status().open_space();
  const int index_history_size =
      open_space_status.partitioned_trajectories_index_history_size();

  if (index_history_size <= 1) {
    return false;
  }
  for (int i = 0; i < index_history_size - 1; i++) {
    const auto& index_history =
        open_space_status.partitioned_trajectories_index_history(i);
    if (index_history == trajectory_encoding_to_check) {
      return true;
    }
  }
  return false;
}

void OpenSpaceTrajectoryPostProcess::UpdateTrajHistory(
    const std::string& chosen_trajectory_encoding) {
  auto* open_space_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_open_space();

  const auto& trajectory_history =
      injector_->planning_context()
          ->planning_status()
          .open_space()
          .partitioned_trajectories_index_history();
  if (trajectory_history.empty()) {
    open_space_status->add_partitioned_trajectories_index_history(
        chosen_trajectory_encoding);
    return;
  }
  if (*(trajectory_history.rbegin()) == chosen_trajectory_encoding) {
    return;
  }
  open_space_status->add_partitioned_trajectories_index_history(
      chosen_trajectory_encoding);
}

void OpenSpaceTrajectoryPostProcess::PartitionTrajectory(
    const DiscretizedTrajectory& raw_trajectory,
    std::vector<TrajGearPair>* partitioned_trajectories) {
  CHECK_NOTNULL(partitioned_trajectories);

  size_t horizon = raw_trajectory.size();

  partitioned_trajectories->clear();
  partitioned_trajectories->emplace_back();
  TrajGearPair* current_trajectory_gear = &(partitioned_trajectories->back());

  auto* trajectory = &(current_trajectory_gear->first);
  auto* gear = &(current_trajectory_gear->second);

  // Decide initial gear
  const auto& first_path_point = raw_trajectory.front().path_point();
  const auto& second_path_point = raw_trajectory[1].path_point();
  double heading_angle = first_path_point.theta();
  const Vec2d init_tracking_vector(
      second_path_point.x() - first_path_point.x(),
      second_path_point.y() - first_path_point.y());
  double tracking_angle = init_tracking_vector.Angle();
  *gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
              (M_PI_2)
          ? canbus::Chassis::GEAR_DRIVE
          : canbus::Chassis::GEAR_REVERSE;

  // Set accumulated distance
  Vec2d last_pos_vec(first_path_point.x(), first_path_point.y());
  double distance_s = 0.0;
  bool is_trajectory_last_point = false;

  for (size_t i = 0; i < horizon - 1; ++i) {
    const TrajectoryPoint& trajectory_point = raw_trajectory.at(i);
    const TrajectoryPoint& next_trajectory_point = raw_trajectory.at(i + 1);

    // Check gear change
    heading_angle = trajectory_point.path_point().theta();
    const Vec2d tracking_vector(next_trajectory_point.path_point().x() -
                                    trajectory_point.path_point().x(),
                                next_trajectory_point.path_point().y() -
                                    trajectory_point.path_point().y());
    tracking_angle = tracking_vector.Angle();
    auto cur_gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
                (M_PI_2)
            ? canbus::Chassis::GEAR_DRIVE
            : canbus::Chassis::GEAR_REVERSE;

    if (cur_gear != *gear) {
      is_trajectory_last_point = true;
      LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                          &last_pos_vec, &distance_s, trajectory);
      partitioned_trajectories->emplace_back();
      current_trajectory_gear = &(partitioned_trajectories->back());
      current_trajectory_gear->second = cur_gear;
      distance_s = 0.0;
      is_trajectory_last_point = false;
    }

    trajectory = &(current_trajectory_gear->first);
    gear = &(current_trajectory_gear->second);

    LoadTrajectoryPoint(trajectory_point, is_trajectory_last_point, *gear,
                        &last_pos_vec, &distance_s, trajectory);
  }
  is_trajectory_last_point = true;
  const TrajectoryPoint& last_trajectory_point = raw_trajectory.back();
  LoadTrajectoryPoint(last_trajectory_point, is_trajectory_last_point, *gear,
                      &last_pos_vec, &distance_s, trajectory);
}

void OpenSpaceTrajectoryPostProcess::LoadTrajectoryPoint(
    const TrajectoryPoint& trajectory_point,
    const bool is_trajectory_last_point,
    const canbus::Chassis::GearPosition& gear, Vec2d* last_pos_vec,
    double* distance_s, DiscretizedTrajectory* current_trajectory) {
  current_trajectory->emplace_back();
  TrajectoryPoint* point = &(current_trajectory->back());
  point->set_relative_time(trajectory_point.relative_time());
  point->mutable_path_point()->set_x(trajectory_point.path_point().x());
  point->mutable_path_point()->set_y(trajectory_point.path_point().y());
  point->mutable_path_point()->set_theta(trajectory_point.path_point().theta());
  point->set_v(trajectory_point.v());
  point->mutable_path_point()->set_s(*distance_s);
  Vec2d cur_pos_vec(trajectory_point.path_point().x(),
                    trajectory_point.path_point().y());
  *distance_s += (gear == canbus::Chassis::GEAR_REVERSE ? -1.0 : 1.0) *
                 (cur_pos_vec.DistanceTo(*last_pos_vec));
  *last_pos_vec = cur_pos_vec;
  point->mutable_path_point()->set_kappa((is_trajectory_last_point ? -1 : 1) *
                                         std::tan(trajectory_point.steer()) /
                                         wheel_base_);
  point->set_a(trajectory_point.a());

  AINFO << "Load trajectory point: " << point->DebugString();
}

bool OpenSpaceTrajectoryPostProcess::CheckReachTrajectoryEnd(
    const DiscretizedTrajectory& trajectory,
    const canbus::Chassis::GearPosition& gear, const size_t trajectories_size,
    size_t* current_trajectory_point_index) {
  // Check if have reached endpoint of trajectory
  const TrajectoryPoint& trajectory_end_point = trajectory.back();
  const size_t trajectory_size = trajectory.size();
  const PathPoint& path_end_point = trajectory_end_point.path_point();
  AINFO << "scale_destination_:" << scale_destination_;
  double scale = current_trajectory_index_ == frame_->open_space_info().partitioned_trajectories().size() - 1
          ? scale_destination_
          : 1.0;
  AINFO << "Scale: " << scale;
  if (CheckArrivePoint(
        gear,
        path_end_point,
        scale * lateral_offset_to_midpoint_,
        scale * longitudinal_offset_to_midpoint_,
        scale * heading_offset_to_midpoint_)) {
    if (current_trajectory_index_ + 1 >= trajectories_size) {
      current_trajectory_index_ = trajectories_size - 1;
      *current_trajectory_point_index = trajectory_size - 1;
    } else {
      current_trajectory_index_ += 1;
      *current_trajectory_point_index = 0;
    }
    stop_check_count_ = 0;
    AINFO << "Reach the end of a trajectory, switching to next one";
    return true;
  } else {
    return false;
  }
}

bool OpenSpaceTrajectoryPostProcess::UseFailSafeSearch(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const std::vector<std::string>& trajectories_encodings,
    size_t* current_trajectory_index, size_t* current_trajectory_point_index) {
  AERROR << "Trajectory partition fail, using failsafe search";
  const size_t trajectories_size = partitioned_trajectories.size();
  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      failsafe_closest_point_on_trajs;
  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = partitioned_trajectories.at(i).first;
    size_t trajectory_size = trajectory.size();
    CHECK_GT(trajectory_size, 0U);
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>
        failsafe_closest_point;

    for (size_t j = 0; j < trajectory_size; ++j) {
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
      const double distance = tracking_vector.Length();
      if (distance < distance_search_range_) {
        // get vehicle box and path point box, compute IOU
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length_, ego_width_);
        Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                        shift_distance_ * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
        failsafe_closest_point.emplace(j, iou_ratio);
      }
    }
    if (!failsafe_closest_point.empty()) {
      size_t closest_point_index = failsafe_closest_point.top().first;
      double max_iou_ratio = failsafe_closest_point.top().second;
      failsafe_closest_point_on_trajs.emplace(
          std::make_pair(i, closest_point_index), max_iou_ratio);
    }
  }
  if (failsafe_closest_point_on_trajs.empty()) {
    return false;
  } else {
    bool closest_and_not_repeated_traj_found = false;
    while (!failsafe_closest_point_on_trajs.empty()) {
      *current_trajectory_index =
          failsafe_closest_point_on_trajs.top().first.first;
      *current_trajectory_point_index =
          failsafe_closest_point_on_trajs.top().first.second;
      if (CheckTrajTraversed(
              trajectories_encodings[*current_trajectory_index])) {
        failsafe_closest_point_on_trajs.pop();
      } else {
        closest_and_not_repeated_traj_found = true;
        UpdateTrajHistory(trajectories_encodings[*current_trajectory_index]);
        return true;
      }
    }
    if (!closest_and_not_repeated_traj_found) {
      return false;
    }

    return true;
  }
}

bool OpenSpaceTrajectoryPostProcess::InsertGearShiftTrajectory(
    const bool flag_change_to_next, const size_t current_trajectory_index,
    const std::vector<TrajGearPair>& partitioned_trajectories,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  const auto* last_frame = injector_->frame_history()->Latest();
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  if (last_frame) {
    const auto& last_gear_status =
        last_frame->open_space_info().gear_switch_states();
    *(current_gear_status) = last_gear_status;
  } else {
    AERROR << "Lost last frame";
  }
  const auto& curr_gear =
      partitioned_trajectories.at(current_trajectory_index).second;
  if (flag_change_to_next || !current_gear_status->gear_shift_period_finished ||
      curr_gear != ego_gear_) {
    current_gear_status->gear_shift_period_finished = false;
    if (current_gear_status->gear_shift_period_started) {
      current_gear_status->gear_shift_start_time =
          Clock::Instance()->NowInSeconds();
      current_gear_status->gear_shift_position =
          partitioned_trajectories.at(current_trajectory_index).second;
      current_gear_status->gear_shift_period_started = false;
      current_gear_status->gear_shift_period_time = 0.0;
    }
    if (current_gear_status->gear_shift_period_time >
            config_.gear_shift_period_duration() &&
        current_gear_status->gear_shift_position == ego_gear_) {
      current_gear_status->gear_shift_period_finished = true;
      current_gear_status->gear_shift_period_started = true;
      stop_check_count_ = 0;
      AINFO << "finished gear shift";
    } else {
      double init_kappa = partitioned_trajectories.at(current_trajectory_index)
                              .first[0]
                              .path_point()
                              .kappa();
      GenerateGearShiftTrajectory(current_gear_status->gear_shift_position,
                                  init_kappa, gear_switch_idle_time_trajectory);
      AINFO << "change gear: " << current_gear_status->gear_shift_position;
      current_gear_status->gear_shift_period_time =
          Clock::Instance()->NowInSeconds() -
          current_gear_status->gear_shift_start_time;
      return true;
    }
  }

  return true;
}

void OpenSpaceTrajectoryPostProcess::GenerateGearShiftTrajectory(
    const canbus::Chassis::GearPosition& gear_position, double init_kappa,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  gear_switch_idle_time_trajectory->first.clear();
  const double gear_shift_max_t = config_.gear_shift_max_t();
  const double gear_shift_unit_t = config_.gear_shift_unit_t();
  // TrajectoryPoint point;
  for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(init_kappa);
    point.set_relative_time(t);
    point.set_v(0.0);
    point.set_a(0.0);
    gear_switch_idle_time_trajectory->first.emplace_back(point);
  }
  ADEBUG << "gear_switch_idle_time_trajectory"
         << gear_switch_idle_time_trajectory->first.size();
  gear_switch_idle_time_trajectory->second = gear_position;
}

void OpenSpaceTrajectoryPostProcess::GenerateStopTrajectory(
    const canbus::Chassis::GearPosition& gear_position,
    TrajGearPair* stop_trajectory) {
  stop_trajectory->first.clear();
  int alpha = gear_position == canbus::Chassis::GEAR_DRIVE ? 1 : -1;
  // TrajectoryPoint point;
  for (double t = 0.0; t < 2.0; t += 0.1) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(t);
    point.set_v(0.0);
    point.set_a(alpha * -0.2);
    stop_trajectory->first.emplace_back(point);
  }
  ADEBUG << "stop_trajectory"
         << stop_trajectory->first.size();
  stop_trajectory->second = gear_position;
}

void OpenSpaceTrajectoryPostProcess::AdjustRelativeTimeAndS(
    const std::vector<TrajGearPair>& partitioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    TrajGearPair* current_partitioned_trajectory) {
  const size_t partitioned_trajectories_size = partitioned_trajectories.size();
  CHECK_GT(partitioned_trajectories_size, current_trajectory_index);

  // Reassign relative time and relative s to have the closest point as origin
  // point
  *(current_partitioned_trajectory) =
      partitioned_trajectories.at(current_trajectory_index);
  auto trajectory = &(current_partitioned_trajectory->first);

  for (int i = 0; i < trajectory->size(); i++) {
    ADEBUG << "trajectory: " << i << " " << std::setprecision(9) <<
    trajectory->at(i).path_point().x() << " " <<
    trajectory->at(i).path_point().y();
  }

  double time_shift =
      trajectory->at(closest_trajectory_point_index).relative_time();
  double s_shift =
      trajectory->at(closest_trajectory_point_index).path_point().s();
  const size_t trajectory_size = trajectory->size();
  for (size_t i = 0; i < trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }

  // Reassign relative t and s on stitched_trajectory_result for accurate next
  // frame stitching
  //   size_t interpolated_pieces_num =
  //       config_.interpolated_pieces_num();
  //   if (FLAGS_use_iterative_anchoring_smoother) {
  //     interpolated_pieces_num = 1;
  //   }
  // const size_t unpartitioned_trajectory_size =
  //     unpartitioned_trajectory_result->size();
  //   size_t index_estimate = 0;
  //   for (size_t i = 0; i < current_trajectory_index; ++i) {
  //     index_estimate += partitioned_trajectories.at(i).first.size();
  //   }
  //   index_estimate += closest_trajectory_point_index;
  //   index_estimate /= interpolated_pieces_num;
  //   if (index_estimate >= unpartitioned_trajectory_size) {
  //     index_estimate = unpartitioned_trajectory_size - 1;
  //   }
  //   time_shift =
  //       unpartitioned_trajectory_result->at(index_estimate).relative_time();
  //   s_shift =
  //       unpartitioned_trajectory_result->at(index_estimate).path_point().s();
  // for (size_t i = 0; i < unpartitioned_trajectory_size; ++i) {
  //   TrajectoryPoint* trajectory_point =
  //       &(unpartitioned_trajectory_result->at(i));
  //   trajectory_point->set_relative_time(trajectory_point->relative_time() -
  //                                       time_shift);
  //   trajectory_point->mutable_path_point()->set_s(
  //       trajectory_point->path_point().s() - s_shift);
  // }
}

bool OpenSpaceTrajectoryPostProcess::CheckArrivePoint(
        const canbus::Chassis::GearPosition& gear,
        const PathPoint& path_end_point,
        const double& lateral_offset_to_midpoint,
        const double& longitudinal_offset_to_midpoint,
        const double& heading_offset_to_midpoint) {
  AINFO << "path_end_point " << path_end_point.DebugString();
  AINFO << "ego_x_: " << ego_x_
      << ", ego_y_: " << ego_y_
      << " heading: " << vehicle_moving_direction_;
  const double path_end_point_x = path_end_point.x();
  const double path_end_point_y = path_end_point.y();
  const Vec2d tracking_vector(ego_x_ - path_end_point_x, ego_y_ - path_end_point_y);
  const double path_end_point_theta = path_end_point.theta();
  const double included_angle = NormalizeAngle(path_end_point_theta - tracking_vector.Angle());
  const double distance_to_trajs_end = std::sqrt(
          (path_end_point_x - ego_x_) * (path_end_point_x - ego_x_)
          + (path_end_point_y - ego_y_) * (path_end_point_y - ego_y_));
  const double lateral_offset = std::abs(distance_to_trajs_end * std::sin(included_angle));
  const double longitudinal_offset = std::abs(distance_to_trajs_end * std::cos(included_angle));
  const double traj_end_point_moving_direction
          = ego_gear_ == canbus::Chassis::GEAR_REVERSE ? NormalizeAngle(path_end_point_theta + M_PI) : path_end_point_theta;

  const double heading_search_to_trajs_end
          = std::abs(NormalizeAngle(traj_end_point_moving_direction - vehicle_moving_direction_));

  // If close to the end point, start on the next trajectory
  double end_point_iou_ratio = 0.0;
  if (lateral_offset < lateral_offset_to_midpoint && longitudinal_offset < longitudinal_offset_to_midpoint
      && heading_search_to_trajs_end < heading_offset_to_midpoint
      && stop_check_count_ > stop_check_window_) {
    // get vehicle box and path point box, compare with a threadhold in IOU
    Box2d path_end_point_box({path_end_point_x, path_end_point_y}, path_end_point_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(path_end_point_theta), shift_distance_ * std::sin(path_end_point_theta)};
    path_end_point_box.Shift(shift_vec);
    end_point_iou_ratio = Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_end_point_box));

    if (end_point_iou_ratio > vehicle_box_iou_threshold_to_midpoint_) {
      AINFO << "ego reach point";
      return true;
    }
  }

  AINFO << "Vehicle did not reach end of a trajectory with conditions for "
           "lateral distance_check: "
        << (lateral_offset < lateral_offset_to_midpoint) << " and actual lateral distance: " << lateral_offset
        << "; longitudinal distance_check: " << (longitudinal_offset < longitudinal_offset_to_midpoint)
        << " and actual longitudinal distance: " << longitudinal_offset
        << "; heading_check: " << (heading_search_to_trajs_end < heading_offset_to_midpoint)
        << " with actual heading: " << heading_search_to_trajs_end
        << "; stop_check: " << (stop_check_count_ > stop_check_window_)
        << " with actual stop count: " << stop_check_count_
        << "; velocity_check: " << (std::abs(ego_v_) < vehicle_param_.max_abs_speed_when_stopped())
        << " with actual linear velocity: " << ego_v_
        << "; iou_check: " << (end_point_iou_ratio > vehicle_box_iou_threshold_to_midpoint_)
        << " with actual iou: " << end_point_iou_ratio;
  return false;
}

// bool OpenSpaceTrajectoryPostProcess::SpeedOptimizer(
//     const std::vector<TrajGearPair>& partitioned_trajectories,
//     const size_t current_trajectory_index,
//     const size_t closest_trajectory_point_index,
//     DiscretizedTrajectory* unpartitioned_trajectory_result,
//     TrajGearPair* current_partitioned_trajectory) {
//   *(current_partitioned_trajectory) =
//       partitioned_trajectories.at(current_trajectory_index);
// }

}  // namespace planning
}  // namespace apollo
