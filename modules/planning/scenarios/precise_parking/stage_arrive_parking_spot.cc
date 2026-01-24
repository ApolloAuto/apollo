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

#include "modules/planning/scenarios/precise_parking/stage_arrive_parking_spot.h"

#include <string>
#include <vector>

#include "modules/planning/planning_base/math/discrete_points_math.h"
#include "modules/planning/planning_open_space/utils/open_space_roi_util.h"
#include "modules/planning/planning_open_space/utils/open_space_fallback_util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

bool StageArriveParkingSpot::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    CHECK_NOTNULL(context);
    bool ret = Stage::Init(config, injector, config_dir, context);
    if (!ret) {
        AERROR << Name() << "init failed!";
        return false;
    }
    precise_parking_context_ = GetContextAs<PreciseParkingContext>();
    return ret;
}

StageResult StageArriveParkingSpot::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    StageResult result;
    if (precise_parking_context_->precise_parking_command.mission_type()
        == external_command::PreciseMissionType::DUMP) {
        AINFO << "Dumping parking spot";
        return ExecuteDumpMission(frame);
    } else if (precise_parking_context_->precise_parking_command.mission_type() == external_command::PreciseMissionType::CHARGE) {
        AINFO << "Charging parking spot";
        return ExecuteChargeMission(frame);
    }
    // else if (
    //         precise_parking_context_->precise_parking_command.mission_type()
    //         == external_command::PreciseParkingCommand::Type::CHARGE) {
    //     AINFO << "charge parking spot";
    //     return ExecuteChargeMission(frame);
    // }
    return result.SetStageStatus(StageStatusType::ERROR);
}

StageResult StageArriveParkingSpot::ExecuteDumpMission(Frame* frame) {
    StageResult result;
    if (retry_mission_) {
        AINFO << "retry parking";
        if (!GenerateRetryParkingTrajectory(frame)) {
            AERROR << "Generate retry parking trajectory failed!";
            return result.SetStageStatus(StageStatusType::ERROR);
        }
    } else if (!GenerateParkingTrajectory(frame)) {
        AERROR << "Generate parking trajectory failed!";
        return result.SetStageStatus(StageStatusType::ERROR);
    }
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
    result = ExecuteTaskOnOpenSpace(frame);

    if (arrive_parking_spot_) {
        AINFO << "Already arrived parking spot";
        CheckParkingAccurate(frame);
        frame->mutable_open_space_info()->set_openspace_planning_finish(true);
        // GenerateStopTrajectory(frame);
        return result.SetStageStatus(StageStatusType::FINISHED);
    }

    if (frame->open_space_info().destination_reached()) {
        AINFO << "StagePreceseParking destination reached";
        
        if (CheckParkingAccurate(frame)) {
            arrive_parking_spot_ = true;
            retry_mission_ = false;
            frame->mutable_open_space_info()->set_openspace_planning_finish(true);
            return result.SetStageStatus(StageStatusType::FINISHED);
        } else {
            AINFO << "arrive but parking imprecision!! retry parking";
            frame->mutable_open_space_info()->mutable_partitioned_trajectories()->clear();
            injector_->frame_history()->Clear();
            retry_mission_ = true;
        }
        
    }

    if (result.HasError()) {
        AERROR << "StageFreeSpace planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageArriveParkingSpot::GenerateParkingTrajectory(Frame* frame) {
    DiscretizedTrajectory opt_trajectory;
    PrintCurves print_debug;
    auto* mutable_open_space_info = frame->mutable_open_space_info();
    double end_heading = precise_parking_context_->precise_parking_command.parking_spot_pose().heading();
    double path_length = precise_parking_context_->scenario_config.precise_trajectory_length();
    Vec2d last_path_point
            = Vec2d(precise_parking_context_->precise_parking_command.parking_spot_pose().x(),
                    precise_parking_context_->precise_parking_command.parking_spot_pose().y());
    Vec2d unit_vec = precise_parking_context_->precise_parking_command.parking_inwards()
            ? path_length * Vec2d(std::cos(end_heading), std::sin(end_heading)) * -1.0
            : path_length * Vec2d(std::cos(end_heading), std::sin(end_heading));
    last_path_point = last_path_point + unit_vec;
    double relative_s = 0.0;
    double total_t = path_length / 0.1;
    double delta_t = 0.1;
    int num_of_points = total_t / delta_t;

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_points, delta_t, {0.0, 0.0, 0.0});
    std::vector<double> x_ref(num_of_points, path_length);
    std::vector<std::pair<double, double>> x_bounds(num_of_points, {0.0, path_length});
    std::vector<std::pair<double, double>> dx_bounds(num_of_points, {0.0, 0.5});
    std::vector<std::pair<double, double>> ddx_bounds(num_of_points, {-4.0, 4.0});
    piecewise_jerk_problem.set_x_ref(10.0, std::move(x_ref));
    piecewise_jerk_problem.set_dx_ref(10.0, 0.5);
    piecewise_jerk_problem.set_weight_ddx(1.0);
    piecewise_jerk_problem.set_weight_dddx(10.0);
    piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
    piecewise_jerk_problem.set_dx_bounds(0.0, 0.5);
    piecewise_jerk_problem.set_ddx_bounds(-4.0, 4.0);
    piecewise_jerk_problem.set_dddx_bound(-2.0, 2.0);
    // solve the problem
    if (!piecewise_jerk_problem.Optimize()) {
        AERROR << "Piecewise jerk speed optimizer failed!";
        return false;
    }

    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

    AINFO << "unit_vec: " << unit_vec.x() << " " << unit_vec.y();
    for (size_t i = 1; i < num_of_points; ++i) {
        print_debug.AddPoint("optimize_st_curve", i * delta_t, s[i]);
        print_debug.AddPoint("optimize_vt_curve", i * delta_t, ds[i]);
        print_debug.AddPoint("optimize_at_curve", i * delta_t, dds[i]);

        common::TrajectoryPoint point;
        if (s[i] - s[i - 1] < 1e-3) {
            AINFO << "skip point";
            continue;
        }
        Vec2d pose = last_path_point - unit_vec / path_length * (s[i] - s[i - 1]);
        // AINFO << "path_length: " << path_length << " s: " << s[i] << " pose: " << pose.x() << " " << pose.y();
        if (std::fabs(s[i] - path_length) < 1e-2) {
            break;
        }
        point.mutable_path_point()->set_x(pose.x());
        point.mutable_path_point()->set_y(pose.y());
        point.mutable_path_point()->set_theta(end_heading);
        if (precise_parking_context_->precise_parking_command.parking_inwards()) {
            point.set_a(dds[i]);
            point.set_v(ds[i]);
            relative_s += pose.DistanceTo(last_path_point);
        } else {
            relative_s -= pose.DistanceTo(last_path_point);
            point.set_a(-dds[i]);
            point.set_v(-ds[i]);
        }
        point.mutable_path_point()->set_s(relative_s);
        point.set_steer(0.0);
        point.set_relative_time(i * 0.1);
        opt_trajectory.emplace_back(point);
        last_path_point = std::move(pose);
    }

    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(last_path_point.x());
    point.mutable_path_point()->set_y(last_path_point.y());
    point.mutable_path_point()->set_theta(end_heading);
    point.set_a(0);
    point.set_v(0);
    if (precise_parking_context_->precise_parking_command.parking_inwards()) {
        relative_s = path_length;
    } else {
        relative_s = -path_length;
    }
    point.mutable_path_point()->set_s(relative_s);
    point.set_steer(0.0);
    point.set_relative_time(num_of_points * 0.1);
    opt_trajectory.emplace_back(point);

    print_debug.PrintToLog();
    *frame->mutable_open_space_info()->mutable_optimizer_trajectory_data() = std::move(opt_trajectory);
    return true;
}

StageResult StageArriveParkingSpot::ExecuteChargeMission(Frame* frame) {
    const auto& dock_info = frame->local_view().perception_dock;
    DiscretizedTrajectory trajectory;
    std::vector<std::pair<double, double>> xy_points(dock_info->relative_path_position().size());
    std::vector<double> headings, accumulated_s, kappas, dkappas;
    xy_points.reserve(dock_info->relative_path_position().size());
    for (const auto& iter : dock_info->relative_path_position()) {
        xy_points.emplace_back(iter.x(), iter.y());
    }
    DiscretePointsMath::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas);
    for (size_t i = 0; i < xy_points.size(); i++)
    {
        common::TrajectoryPoint point;
        point.mutable_path_point()->set_x(xy_points[i].first);
        point.mutable_path_point()->set_y(xy_points[i].second);
        point.mutable_path_point()->set_theta(headings[i]);
        point.mutable_path_point()->set_s(accumulated_s[i]);
        point.mutable_path_point()->set_kappa(kappas[i]);
        point.mutable_path_point()->set_dkappa(dkappas[i]);
        trajectory.emplace_back(point);
    }
    canbus::Chassis::GearPosition gear_position = precise_parking_context_->precise_parking_command.parking_inwards() ? canbus::Chassis::GEAR_DRIVE : canbus::Chassis::GEAR_REVERSE;
    TrajGearPair partition_trajectory = std::make_pair(trajectory, gear_position);

    std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles;
    std::vector<Obstacle*> static_obstacles;
    OpenSpaceFallbackUtil::BuildPredictedEnvironment(Vec2d(frame->vehicle_state().x(), frame->vehicle_state().y()), frame->obstacles(), predicted_bounding_rectangles, 5.0, 10.0);
    OpenSpaceFallbackUtil::BuildStaticObstacleEnvironment(Vec2d(frame->vehicle_state().x(), frame->vehicle_state().y()), frame->obstacles(), static_obstacles, 10.0);

    int current_index = 0;
    int first_collision_index = 0;
    bool is_collision_with_static_obstacle = false;
    bool is_collision_with_dynamic_obstacle = false;
    double collilsion_time_buffer = 0.2;
    if (!OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(partition_trajectory, predicted_bounding_rectangles, static_obstacles, &current_index, &first_collision_index, is_collision_with_static_obstacle, is_collision_with_dynamic_obstacle, collilsion_time_buffer)) {
      frame->mutable_open_space_info()->set_is_collision(true);
    }
    *(frame->mutable_open_space_info()->mutable_chosen_partitioned_trajectory()) = std::move(partition_trajectory);
    StageResult result;
    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageArriveParkingSpot::CheckParkingAccurate(Frame* frame) {
  double ego_x = injector_->vehicle_state()->x();
  double ego_y = injector_->vehicle_state()->y();
  double ego_theta = injector_->vehicle_state()->heading();
  std::vector<double> end_pose =
      {precise_parking_context_->precise_parking_command.parking_spot_pose().x(),
       precise_parking_context_->precise_parking_command.parking_spot_pose().y(),
       precise_parking_context_->precise_parking_command.parking_spot_pose().heading()};
  double delta_theta = std::fabs(
      common::math::NormalizeAngle(ego_theta - end_pose[2]));
  common::math::Vec2d vec(ego_x - end_pose[0], ego_y - end_pose[1]);
  common::math::Vec2d park_unit_vec = common::math::Vec2d::CreateUnitVec2d(end_pose[2]);
  double lat_error = park_unit_vec.CrossProd(vec);
  double lon_error = park_unit_vec.InnerProd(vec);
  AINFO << "lat error: " << lat_error << " "
        << "lon error: " << lon_error << " "
        << "delta_theta: " << delta_theta;
  if (std::fabs(lat_error) > precise_parking_context_->scenario_config.max_lat_error() ||
      std::fabs(lon_error) > precise_parking_context_->scenario_config.max_lon_error() ||
      delta_theta > precise_parking_context_->scenario_config.max_heading_error()) {
        AINFO << "parking not accurate";
        return false;
  }
  return true;
}

bool StageArriveParkingSpot::GenerateRetryParkingTrajectory(Frame* frame) {
  if (!GenerateParkingTrajectory(frame)) {
      AINFO << "Generate parking trajectory failed";
      return false;
  }
  DiscretizedTrajectory* opt_trajectory_ptr = frame->mutable_open_space_info()->mutable_optimizer_trajectory_data();
  DiscretizedTrajectory trajectory = *opt_trajectory_ptr;
  double total_time = trajectory.back().relative_time();
  reverse(trajectory.begin(), trajectory.end());
  double path_length = precise_parking_context_->scenario_config.precise_trajectory_length();
  for (auto& point : trajectory) {
      point.set_a(-point.a());
      point.set_v(-point.v());
      point.set_relative_time(total_time - point.relative_time());
      if (precise_parking_context_->precise_parking_command.parking_inwards()) {
          point.mutable_path_point()->set_s(path_length - point.path_point().s());
      } else {
          point.mutable_path_point()->set_s(path_length + point.path_point().s());
      }
  }
  for (auto& point : *opt_trajectory_ptr) {
      point.set_relative_time(total_time + point.relative_time());
  }
  opt_trajectory_ptr->insert(opt_trajectory_ptr->begin(), trajectory.begin(), trajectory.end());
  for (auto& point : *opt_trajectory_ptr) {
      AINFO << "retry parking trajectory: " << point.DebugString();
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
