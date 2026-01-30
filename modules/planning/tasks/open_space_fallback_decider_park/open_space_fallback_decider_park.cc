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

#include "modules/planning/tasks/open_space_fallback_decider_park/open_space_fallback_decider_park.h"

namespace apollo {
namespace planning {
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

bool OpenSpaceFallbackDeciderPark::Init(
    const std::string& config_dir, const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!Decider::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  if (!Decider::LoadConfig<OpenSpaceFallBackDeciderParkConfig>(&config_)) {
    AERROR << "Failed to load config file";
    return false;
  } else {
    open_space_fallback_collision_distance_ =
        config_.open_space_fallback_collision_distance();
    return true;
  }
}

bool OpenSpaceFallbackDeciderPark::QuardraticFormulaLowerSolution(const double a,
                                                              const double b,
                                                              const double c,
                                                              double* sol) {
  // quardratic formula: ax^2 + bx + c = 0, return lower solution
  // TODO(QiL): use const from common::math
  const double kEpsilon = 1e-6;
  *sol = 0.0;
  if (std::abs(a) < kEpsilon) {
    return false;
  }

  double tmp = b * b - 4 * a * c;
  if (tmp < kEpsilon) {
    return false;
  }
  double sol1 = (-b + std::sqrt(tmp)) / (2.0 * a);
  double sol2 = (-b - std::sqrt(tmp)) / (2.0 * a);

  *sol = std::abs(std::min(sol1, sol2));
  ADEBUG << "QuardraticFormulaLowerSolution finished with sol: " << *sol
         << "sol1: " << sol1 << ", sol2: " << sol2 << "a: " << a << "b: " << b
         << "c: " << c;
  return true;
}

Status OpenSpaceFallbackDeciderPark::Process(Frame* frame) {
  AINFO << "frame->open_space_info().chosen_partitioned_trajectory(): " <<
        frame->open_space_info().chosen_partitioned_trajectory().first.size();
  AINFO << "frame->open_space_info().optimizer_trajectory_data(): " <<
        frame->open_space_info().optimizer_trajectory_data().size();

  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles;
  std::vector<Obstacle*> static_obstacles;
  int first_collision_index = 0;
  int fallback_start_index = 0;
  // mayaochang add
  bool is_collision_with_static_obstacle = false;
  bool is_collision_with_dynamic_obstacle = false;

  // only predicted dynamic obstacle
  OpenSpaceFallbackUtil::BuildPredictedEnvironment(
      Vec2d(frame->vehicle_state().x(), frame->vehicle_state().y()),
      frame->obstacles(),
      predicted_bounding_rectangles,
      config_.open_space_prediction_time_period(),
      config_.collision_check_range());
  // get the static obstacles
  OpenSpaceFallbackUtil::BuildStaticObstacleEnvironment(
      Vec2d(frame->vehicle_state().x(), frame->vehicle_state().y()),
      frame->obstacles(),
      static_obstacles,
      config_.collision_check_range());
  ADEBUG << "Numbers of obstsacles are: " << frame->obstacles().size();
  ADEBUG << "Numbers of predicted bounding rectangles of dynamic obstacle are: "
         << predicted_bounding_rectangles[0].size()
         << " and : " << predicted_bounding_rectangles.size()
         << "Numbers of static obstsacles are:" << static_obstacles.size();

  if (frame->open_space_info().chosen_partitioned_trajectory().first.empty()) {
    AERROR << "No trajectory is generated for fallback";

    // set relative time
    static constexpr double relative_stop_time = 0.1;

    auto fallback_tra_pair =
        frame_->mutable_open_space_info()->
            mutable_chosen_partitioned_trajectory();
    fallback_tra_pair->first.clear();
    auto current_speed = injector_->vehicle_state()->
        vehicle_state().linear_velocity();
    AINFO << "open_space_fallback_collision_distance_: " << open_space_fallback_collision_distance_;
    double stop_deceleration_for_static_collision =
        std::max(std::min(-1 * current_speed * current_speed /
                      (2 * (open_space_fallback_collision_distance_ - 1e-12)),
                      -0.2),
                  -4.0);
    AINFO << "stop_deceleration_for_static_collision: " << stop_deceleration_for_static_collision;
    TrajGearPair fallback_trajectory_pair_candidate;
    fallback_trajectory_pair_candidate.second =
        frame_->local_view().chassis->gear_location();
    CalculateFallbackTrajectory(fallback_trajectory_pair_candidate,
                                stop_deceleration_for_static_collision,
                                relative_stop_time, fallback_tra_pair);
    open_space_fallback_collision_distance_ = std::max (1e-6,
        open_space_fallback_collision_distance_ - fabs(fallback_tra_pair->first[1].path_point().s()));
    AINFO << "open_space_fallback_collision_ditance_: " << open_space_fallback_collision_distance_;
    // check if the fallback trajectory has collision with obstacle
    TrajGearPair fallback_trajectory =
        frame_->open_space_info().chosen_partitioned_trajectory();
    if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
            fallback_trajectory, predicted_bounding_rectangles,
            static_obstacles, &fallback_start_index, &first_collision_index,
            is_collision_with_static_obstacle,
            is_collision_with_dynamic_obstacle,
            config_.open_space_fallback_collision_time_buffer())) {
      AINFO << " fallback trajectory still has collision with obstacles";
      // stop vehicle with max deceleration
      CalculateFallbackTrajectory(fallback_trajectory, -4.0,
                                  relative_stop_time, fallback_tra_pair);
    }
    return Status::OK();
  }

  // set collision distance with static collision
  open_space_fallback_collision_distance_ =
        config_.open_space_fallback_collision_distance();

  if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_partitioned_trajectory(),
          predicted_bounding_rectangles, static_obstacles,
          &fallback_start_index, &first_collision_index,
          is_collision_with_static_obstacle,
          is_collision_with_dynamic_obstacle,
          config_.open_space_fallback_collision_time_buffer())) {
    AINFO << "trajectory still has collision with obstacles";
    // generate fallback trajectory base on current partition trajectory
    // vehicle speed is decreased to zero inside safety distance
    TrajGearPair fallback_trajectory_pair_candidate =
        frame->open_space_info().chosen_partitioned_trajectory();
    // auto* ptr_fallback_trajectory_pair =
    // frame_->mutable_open_space_info()->mutable_fallback_trajectory();
    const auto future_collision_point =
        fallback_trajectory_pair_candidate.first[first_collision_index];

    // Fallback starts from current location but with vehicle velocity
    auto fallback_start_point =
        fallback_trajectory_pair_candidate.first[fallback_start_index];
    const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();

    // TODO(QiL): move 1.0 to configs
    double stop_distance =
        fallback_trajectory_pair_candidate.second == canbus::Chassis::GEAR_DRIVE
            ? std::max(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() - config_.distance_to_obs(),
                       0.0)
            : std::min(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() + config_.distance_to_obs(),
                       0.0);

    ADEBUG << "stop distance : " << stop_distance;

    const auto& vehicle_param =
        common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    const double max_adc_stop_speed =
        vehicle_param.max_abs_speed_when_stopped();

    // static collision action
    if (is_collision_with_static_obstacle) {
      AINFO << "collision with static obstacle";

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

      // set the collision distance with static collision and relative time
      const auto& open_space_fallback_collision_distance =
          config_.open_space_fallback_collision_distance();
      static constexpr double relative_stop_time = 0.1;

      double relative_time = 0.0;
      double current_s_distance = 0.0;
      auto fallback_tra_pair =
          frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory();
      fallback_tra_pair->first.clear();
      double stop_deceleration_for_static_collision = 0;

      auto current_speed = vehicle_state.linear_velocity();
      AINFO << "open_space_fallback_collision_distance: "
            << open_space_fallback_collision_distance
            << "stop_distance: " << stop_distance;
      if (open_space_fallback_collision_distance > std::abs(stop_distance)) {
        AINFO << "The collision distance with static obstacle is too small, "
                  "distance is :"
               << stop_distance << "!";

        stop_deceleration_for_static_collision = -4.0;
      } else {
        AINFO << "The collision distance with static obstacle is large, stop "
                  "with set distance is :"
               << open_space_fallback_collision_distance << "!";
        // generate stop deceleration
        stop_deceleration_for_static_collision =
            std::max(-1 * current_speed * current_speed /
                         (2 * (open_space_fallback_collision_distance - 1e-6)),
                     -4.0);
        AINFO << "current_speed: " << current_speed
              << "open_space_fallback_collision_distance: "
              << open_space_fallback_collision_distance;
      }
      AINFO << "stop deceleration for static collision is : "
            << stop_deceleration_for_static_collision
            << "relative_stop_time: " << relative_stop_time;
      if (config_.is_use_trajectory_pose()) {
        CalculateFallbackTrajectoryWithTrajectoryPose(
                                  fallback_trajectory_pair_candidate,
                                  fallback_start_point, future_collision_point, stop_distance,
                                  relative_stop_time, fallback_tra_pair);
      } else {
        CalculateFallbackTrajectory(fallback_trajectory_pair_candidate,
                                  open_space_fallback_collision_distance,
                                  relative_stop_time, fallback_tra_pair);
      }
      AINFO << "fallback trajectory has collision with obstacles";
      // check if the fallback trajectory has collision with obstacle
      TrajGearPair fallback_trajectory =
          frame_->open_space_info().chosen_partitioned_trajectory();
      AINFO << "fallback trajectory has collision with obstacles";
      if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
              fallback_trajectory, predicted_bounding_rectangles,
              static_obstacles, &fallback_start_index, &first_collision_index,
              is_collision_with_static_obstacle,
              is_collision_with_dynamic_obstacle,
              config_.open_space_fallback_collision_time_buffer())) {
        AINFO << " fallback trajectory still has collision with obstacles";
        // stop vehicle with max deceleration
        CalculateFallbackTrajectory(fallback_trajectory, -4.0,
                                    relative_stop_time, fallback_tra_pair);
      }
    }

    // dynamic collision action
    if (is_collision_with_dynamic_obstacle) {
      auto fallback_tra_pair =
          frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory();
      fallback_tra_pair->first.clear();

      fallback_start_point.set_v(vehicle_state.linear_velocity());

      *(frame_->mutable_open_space_info()->mutable_future_collision_point()) =
          future_collision_point;

      // min stop distance: (max_acc)
      double min_stop_distance =
          0.5 * fallback_start_point.v() * fallback_start_point.v() / 4.0;

      // const auto& vehicle_config =
      //     common::VehicleConfigHelper::Instance()->GetConfig();
      const double vehicle_max_acc = 4.0;  // vehicle_config.max_acceleration();
      const double vehicle_max_dec =
          -4.0;  // vehicle_config.max_deceleration();

      double stop_deceleration = 0.0;

      if (fallback_trajectory_pair_candidate.second ==
          canbus::Chassis::GEAR_REVERSE) {
        stop_deceleration =
            std::min(fallback_start_point.v() * fallback_start_point.v() /
                         (2.0 * (stop_distance + 1e-6)),
                     vehicle_max_acc);
        stop_distance = std::min(-1 * min_stop_distance, stop_distance);
      } else {
        stop_deceleration =
            std::max(-fallback_start_point.v() * fallback_start_point.v() /
                         (2.0 * (stop_distance + 1e-6)),
                     vehicle_max_dec);
        stop_distance = std::max(min_stop_distance, stop_distance);
      }

      AINFO << "stop_deceleration: " << stop_deceleration;

      // Search stop index in chosen trajectory by distance
      int stop_index = fallback_start_index;

      for (int i = fallback_start_index;
           i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
        if (std::abs(
                fallback_trajectory_pair_candidate.first[i].path_point().s()) >=
            std::abs(fallback_start_point.path_point().s() + stop_distance)) {
          stop_index = i;
          break;
        }
      }

      AINFO << "stop index before is: " << stop_index
             << "; fallback_start index before is: " << fallback_start_index;

      for (int i = 0; i < fallback_start_index; ++i) {
        fallback_trajectory_pair_candidate.first[i].set_v(
            fallback_start_point.v());
        fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);
      }

      // TODO(QiL): refine the logic and remove redundant code, change 0.5 to
      // from loading optimizer configs

      // If stop_index == fallback_start_index;
      if (fallback_start_index >= stop_index) {
        // 1. Set fallback start speed to 0, acceleration to max acceleration.
        AINFO << "Stop distance within safety buffer, stop now!";
        fallback_start_point.set_v(0.0);
        fallback_start_point.set_a(0.0);
        fallback_trajectory_pair_candidate.first[stop_index].set_v(0.0);
        fallback_trajectory_pair_candidate.first[stop_index].set_a(0.0);

        // 2. Trim all trajectory points after stop index
        fallback_trajectory_pair_candidate.first.erase(
            fallback_trajectory_pair_candidate.first.begin() + stop_index + 1,
            fallback_trajectory_pair_candidate.first.end());

        // 3. AppendTrajectoryPoint with same location but zero velocity
        for (int i = 0; i < 20; ++i) {
          common::TrajectoryPoint trajectory_point(
              fallback_trajectory_pair_candidate.first[stop_index]);
          trajectory_point.set_relative_time(
              i * 0.5 + 0.5 +
              fallback_trajectory_pair_candidate.first[stop_index]
                  .relative_time());
          fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(
              trajectory_point);
        }

        *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
            fallback_trajectory_pair_candidate;

        return Status::OK();
      }

      AINFO << "before change, size : "
             << fallback_trajectory_pair_candidate.first.size()
             << ", first index information : "
             << fallback_trajectory_pair_candidate.first[0].DebugString()
             << ", second index information : "
             << fallback_trajectory_pair_candidate.first[1].DebugString();

      // If stop_index > fallback_start_index

      for (int i = fallback_start_index; i <= stop_index; ++i) {
        double new_relative_time = 0.0;
        double temp_v = 0.0;
        double c =
            -2.0 * fallback_trajectory_pair_candidate.first[i].path_point().s();

        if (QuardraticFormulaLowerSolution(stop_deceleration,
                                           2.0 * fallback_start_point.v(), c,
                                           &new_relative_time) &&
            std::abs(
                fallback_trajectory_pair_candidate.first[i].path_point().s()) <=
                std::abs(stop_distance)) {
          ADEBUG << "new_relative_time" << new_relative_time;
          temp_v =
              fallback_start_point.v() + stop_deceleration * new_relative_time;
          // speed limit
          if (std::abs(temp_v) < 1.0) {
            fallback_trajectory_pair_candidate.first[i].set_v(temp_v);
          } else {
            fallback_trajectory_pair_candidate.first[i].set_v(
                temp_v / std::abs(temp_v) * 1.0);
          }
          fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);

          fallback_trajectory_pair_candidate.first[i].set_relative_time(
              new_relative_time);
        } else {
          if (i != 0) {
            fallback_trajectory_pair_candidate.first[i]
                .mutable_path_point()
                ->CopyFrom(fallback_trajectory_pair_candidate.first[i - 1]
                               .path_point());
            fallback_trajectory_pair_candidate.first[i].set_v(0.0);
            fallback_trajectory_pair_candidate.first[i].set_a(0.0);
            fallback_trajectory_pair_candidate.first[i].set_relative_time(
                fallback_trajectory_pair_candidate.first[i - 1]
                    .relative_time() +
                0.5);
          } else {
            fallback_trajectory_pair_candidate.first[i].set_v(0.0);
            fallback_trajectory_pair_candidate.first[i].set_a(0.0);
          }
        }
      }

      ADEBUG << "fallback start point after changes: "
             << fallback_start_point.DebugString();

      ADEBUG << "stop index: " << stop_index;
      ADEBUG << "fallback start index: " << fallback_start_index;

      // 2. Erase afterwards
      fallback_trajectory_pair_candidate.first.erase(
          fallback_trajectory_pair_candidate.first.begin() + stop_index + 1,
          fallback_trajectory_pair_candidate.first.end());

      // 3. AppendTrajectoryPoint with same location but zero velocity
      for (int i = 0; i < 20; ++i) {
        common::TrajectoryPoint trajectory_point(
            fallback_trajectory_pair_candidate.first[stop_index]);
        trajectory_point.set_relative_time(
            i * 0.5 + 0.5 +
            fallback_trajectory_pair_candidate.first[stop_index]
                .relative_time());
        fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(
            trajectory_point);
      }
      *(frame_->mutable_open_space_info()->
          mutable_chosen_partitioned_trajectory()) =
          fallback_trajectory_pair_candidate;
    }
  }
  return Status::OK();
}

bool OpenSpaceFallbackDeciderPark::IsCollisionFreeEgoBox() {
  // prediction time resolution: FLAGS_trajectory_time_resolution
  const auto& vehicle_state = frame_->vehicle_state();
  double x = vehicle_state.x();
  double y = vehicle_state.y();
  double heading = vehicle_state.heading();

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();
  Box2d ego_box({x, y}, heading, ego_length, ego_width);
  Polygon2d ego_polygon = Polygon2d(ego_box);

  for (const Obstacle* obstacle : frame_->obstacles()) {
    if (obstacle->IsVirtual() || !obstacle->IsStatic() ||
        obstacle->Perception().width() < 0.2 ||
        obstacle->Perception().length() < 0.2) {
      continue;
    }
    Polygon2d obstacle_polygon = obstacle->PerceptionPolygon();
    if (ego_polygon.HasOverlap(obstacle_polygon)) {
      return false;
    }
  }
  return true;
}

void OpenSpaceFallbackDeciderPark::CalculateFallbackTrajectory(
    const TrajGearPair& ChosenPartitionedTrajectory, const double& deceleration,
    const double& relative_time_interval, TrajGearPair* traj_gear_pair) {
  // clear the trajectory
  traj_gear_pair->first.clear();
  // get the vehicle  current speed
  double current_speed = fabs(frame_->vehicle_state().linear_velocity());
  double pre_speed = current_speed;
  double decelerate = fabs(deceleration);

  double current_s_distance = 0;
  double relative_time = 0;
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();

  // if the current gear is Reverse,Current heading Should be + M_PI when
  // calculate the position
  double moving_heading = frame_->vehicle_state().heading();
  int alpha =
      frame_->local_view().chassis->gear_location()
          == canbus::Chassis::GEAR_DRIVE ? 1 : -1;

  while (pre_speed >
         vehicle_config.vehicle_param().max_abs_speed_when_stopped()) {
    TrajectoryPoint point;
    static constexpr double kSpeedEpsilon = 1e-9;

    point.mutable_path_point()->set_x(frame_->vehicle_state().x() +
                                      current_s_distance * alpha *
                                          std::cos(moving_heading));
    point.mutable_path_point()->set_y(frame_->vehicle_state().y() +
                                      current_s_distance * alpha *
                                          std::sin(moving_heading));
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(current_s_distance * alpha);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(current_speed* alpha);
    point.set_a(decelerate* -1 * alpha);
    traj_gear_pair->first.emplace_back(point);
    relative_time += relative_time_interval;
    pre_speed = current_speed;
    current_speed -= relative_time_interval * decelerate;
    current_s_distance +=
        (pre_speed + current_speed) / 2 * relative_time_interval;
    if (pre_speed < kSpeedEpsilon) break;
  }
  traj_gear_pair->second = frame_->local_view().chassis->gear_location();
  if (traj_gear_pair->first.empty()) {
    for (int i = 0; i < 20; i++) {
      TrajectoryPoint point;
      point.mutable_path_point()->set_x(frame_->vehicle_state().x());
      point.mutable_path_point()->set_y(frame_->vehicle_state().y());
      point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
      point.mutable_path_point()->set_s(0);
      point.mutable_path_point()->set_kappa(0.0);
      point.set_relative_time(relative_time);
      point.set_v(0);
      point.set_a(-1.0 * alpha);
      traj_gear_pair->first.emplace_back(point);
      relative_time += relative_time_interval;
    }
    traj_gear_pair->second = frame_->local_view().chassis->gear_location();
  }
}

void OpenSpaceFallbackDeciderPark::CalculateFallbackTrajectoryWithTrajectoryPose(
    const TrajGearPair& ChosenPartitionedTrajectory, const TrajectoryPoint start_point, const TrajectoryPoint collision_point, const double& stop_distance,
    const double& relative_time_interval, TrajGearPair* traj_gear_pair) {
  static int count = 0;
  AINFO << "stop_distance:" << stop_distance;
  if (std::fabs(stop_distance) >= 1e-3) {
    count ++;
    int alpha = stop_distance < 0 ? -1 : 1;
    double v_max = 0.5;
    double t  = std::fabs(stop_distance) / v_max * 2;
    double a = v_max / t;
    Vec2d unit_vec(collision_point.path_point().x() - start_point.path_point().x(), collision_point.path_point().y() - start_point.path_point().y());
    Vec2d start_vec(start_point.path_point().x(), start_point.path_point().y());
    unit_vec.Normalize();
    for (double relative_t = 0; relative_t < t; relative_t += relative_time_interval){
      TrajectoryPoint point;
      double point_v, point_a, point_s;
      point_v = a * (t - relative_t);
      point_a = -1 * a;
      point_s = std::fabs(stop_distance) - 0.5 * point_v * (t - relative_t);
      Vec2d point_vec = start_vec + unit_vec * point_s;
      point.mutable_path_point()->set_x(point_vec.x());
      point.mutable_path_point()->set_y(point_vec.y());
      point.mutable_path_point()->set_theta(start_point.path_point().theta());
      point.mutable_path_point()->set_s(point_s * alpha);
      point.mutable_path_point()->set_kappa(0.0);
      point.set_relative_time(relative_t - 0.1 * count);
      point.set_v(point_v * alpha);
      point.set_a(point_a * alpha);
      AINFO << "point: " << point.DebugString();
      traj_gear_pair->first.emplace_back(point);
    }
    TrajectoryPoint point;
    Vec2d point_vec = start_vec + unit_vec * std::fabs(stop_distance);
    point.mutable_path_point()->set_x(point_vec.x());
    point.mutable_path_point()->set_y(point_vec.y());
    point.mutable_path_point()->set_theta(start_point.path_point().theta());
    point.mutable_path_point()->set_s(stop_distance);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(t - 0.1 * count);
    point.set_v(0);
    point.set_a(0);
    AINFO << "point: " << point.DebugString();
    traj_gear_pair->first.emplace_back(point);
    traj_gear_pair->second = ChosenPartitionedTrajectory.second;
  }
  if (traj_gear_pair->first.empty()) {
    double relative_time = 0;
    int alpha =
      frame_->local_view().chassis->gear_location()
          == canbus::Chassis::GEAR_DRIVE ? 1 : -1;
    for (int i = 0; i < 20; i++) {
      TrajectoryPoint point;
      point.mutable_path_point()->set_x(frame_->vehicle_state().x());
      point.mutable_path_point()->set_y(frame_->vehicle_state().y());
      point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
      point.mutable_path_point()->set_s(0);
      point.mutable_path_point()->set_kappa(0.0);
      point.set_relative_time(relative_time);
      point.set_v(0);
      point.set_a(-1.0 * alpha);
      traj_gear_pair->first.emplace_back(point);
      relative_time += relative_time_interval;
    }
    traj_gear_pair->second = frame_->local_view().chassis->gear_location();
  }
}

}  // namespace planning
}  // namespace apollo
