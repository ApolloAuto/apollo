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

#include "modules/planning/open_space_planning.h"
#include "modules/planning/planner/open_space/open_space_planner.h"

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::dreamview::Chart;
using apollo::hdmap::HDMapUtil;
using apollo::planning_internal::OpenSpaceDebug;
using apollo::routing::RoutingResponse;

namespace {

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

bool IsDifferentRouting(const RoutingResponse& first,
                        const RoutingResponse& second) {
  if (first.has_header() && second.has_header()) {
    if (first.header().sequence_num() != second.header().sequence_num()) {
      return true;
    }
    if (first.header().timestamp_sec() != second.header().timestamp_sec()) {
      return true;
    }
    return false;
  } else {
    return true;
  }
}
}  // namespace

OpenSpacePlanning::~OpenSpacePlanning() {
  planner_->Stop();
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  FrameHistory::Instance()->Clear();
  last_routing_.Clear();
}

std::string OpenSpacePlanning::Name() const { return "open_space_planning"; }

Status OpenSpacePlanning::Init(const PlanningConfig& config) {
  if (!CheckPlanningConfig(config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config.DebugString());
  }

  PlanningBase::Init(config);

  planner_dispatcher_->Init();

  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map";

  // dispatch planner
  planner_ = planner_dispatcher_->DispatchPlanner();
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config.DebugString());
  }

  start_time_ = Clock::NowInSeconds();

  AINFO << "Open Space Planner Init Done";

  return planner_->Init(config);
}

Status OpenSpacePlanning::InitFrame(const uint32_t sequence_num,
                                    const TrajectoryPoint& planning_start_point,
                                    const double start_time,
                                    const VehicleState& vehicle_state,
                                    ADCTrajectory* output_trajectory) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         start_time, vehicle_state, output_trajectory));
  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  auto status = frame_->InitForOpenSpace();

  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }

  ADEBUG << "Open Space Planner Init Frame Done";

  return Status::OK();
}

void OpenSpacePlanning::RunOnce(const LocalView& local_view,
                                ADCTrajectory* const ptr_trajectory_pb) {
  local_view_ = local_view;
  const double start_timestamp = Clock::NowInSeconds();

  // localization
  ADEBUG << "Get localization:"
         << local_view_.localization_estimate->DebugString();

  // chassis
  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

  Status status = VehicleStateProvider::Instance()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);

  VehicleState vehicle_state =
      VehicleStateProvider::Instance()->vehicle_state();
  DCHECK_GE(start_timestamp, vehicle_state.timestamp());

  // estimate (x, y) at current timestamp
  // This estimate is only valid if the current time and vehicle state timestamp
  // differs only a small amount (20ms). When the different is too large, the
  // estimation is invalid.
  if (FLAGS_estimate_current_vehicle_state &&
      start_timestamp - vehicle_state.timestamp() < 0.020) {
    auto future_xy = VehicleStateProvider::Instance()->EstimateFuturePosition(
        start_timestamp - vehicle_state.timestamp());
    vehicle_state.set_x(future_xy.x());
    vehicle_state.set_y(future_xy.y());
    vehicle_state.set_timestamp(start_timestamp);
  }
  if (!IsVehicleStateValid(vehicle_state)) {
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    return;
  }

  if (IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    // TODO(QiL): Get latest parking info from new routing
  }

  const double planning_cycle_time = FLAGS_open_space_planning_period;

  std::vector<TrajectoryPoint> stitching_trajectory;
  std::string replan_reason;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get(), &replan_reason);

  const size_t frame_num = seq_num_++;
  status =
      InitFrame(static_cast<uint32_t>(frame_num), stitching_trajectory.back(),
                start_timestamp, vehicle_state, ptr_trajectory_pb);

  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  if (!status.ok()) {
    AINFO << status.ToString();
    if (FLAGS_publish_estop) {
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      FillPlanningPb(start_timestamp, &estop_trajectory);
      ptr_trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    }

    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    frame_->mutable_trajectory()->CopyFrom(*ptr_trajectory_pb);
    const uint32_t n = frame_->SequenceNum();
    FrameHistory::Instance()->Add(n, std::move(frame_));
    return;
  }

  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

  ADEBUG << "Planning status:" << status.ToString();

  const auto time_diff_ms = (Clock::NowInSeconds() - start_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << ptr_trajectory_pb->latency_stats().DebugString();

  if (!status.ok()) {
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  FillPlanningPb(start_timestamp, ptr_trajectory_pb);
  ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();

  frame_->mutable_trajectory()->CopyFrom(*ptr_trajectory_pb);

  const uint32_t n = frame_->SequenceNum();
  FrameHistory::Instance()->Add(n, std::move(frame_));
}

Status OpenSpacePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();

  auto status = dynamic_cast<OpenSpacePlanner&>(*planner_).Plan(
      stitching_trajectory, frame_.get());

  Status trajectory_partition_status;

  if (status == Status::OK()) {
    ADCTrajectory* trajectory_after_stitching_point =
        frame_->mutable_trajectory();
    last_stitching_trajectory_ = frame_->last_stitching_trajectory();
    trajectory_after_stitching_point->mutable_header()->set_timestamp_sec(
        current_time_stamp);

    // adjust the relative time and relative s
    int size_of_trajectory_after_stitching_point =
        trajectory_after_stitching_point->trajectory_point_size();
    double last_stitching_trajectory_relative_time =
        last_stitching_trajectory_.back().relative_time();
    double last_stitching_trajectory_relative_s =
        last_stitching_trajectory_.back().path_point().s();
    for (int i = 0; i < size_of_trajectory_after_stitching_point; i++) {
      trajectory_after_stitching_point->mutable_trajectory_point(i)
          ->set_relative_time(
              trajectory_after_stitching_point->mutable_trajectory_point(i)
                  ->relative_time() +
              last_stitching_trajectory_relative_time);
      trajectory_after_stitching_point->mutable_trajectory_point(i)
          ->mutable_path_point()
          ->set_s(trajectory_after_stitching_point->mutable_trajectory_point(i)
                      ->path_point()
                      .s() +
                  last_stitching_trajectory_relative_s);
    }

    last_publishable_trajectory_.reset(
        new PublishableTrajectory(*trajectory_after_stitching_point));

    ADEBUG << "current_time_stamp: " << std::to_string(current_time_stamp);

    if (FLAGS_enable_stitch_last_trajectory) {
      last_publishable_trajectory_->PrependTrajectoryPoints(
          last_stitching_trajectory_.begin(),
          last_stitching_trajectory_.end() - 1);
    }

    // save the publishable trajectory for use when no planning is generated
    last_trajectory_ = std::make_unique<PublishableTrajectory>(
        current_time_stamp, *last_publishable_trajectory_);
    last_open_space_debug_.CopyFrom(frame_->open_space_debug());
    last_trajectory_succeeded_ = true;

    if (FLAGS_enable_record_debug) {
      ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
          last_stitching_trajectory_.back());
      ADEBUG << "Open space init point added!";
      ptr_debug->mutable_planning_data()->mutable_open_space()->CopyFrom(
          frame_->open_space_debug());
      ADEBUG << "Open space debug information added!";
    }

    // trajectory partition and choose the current trajectory to follow
    trajectory_partition_status =
        TrajectoryPartition(last_publishable_trajectory_, ptr_trajectory_pb);

    if (FLAGS_enable_record_debug && FLAGS_export_chart) {
      FillPlanningPb(current_time_stamp, ptr_trajectory_pb);
      ExportOpenSpaceChart(ptr_debug, ptr_trajectory_pb);
    }

  } else if (status ==
             Status(ErrorCode::OK,
                    "Waiting for planning thread in OpenSpacePlanner")) {
    if (last_trajectory_succeeded_) {
      if (FLAGS_enable_record_debug) {
        ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
            last_stitching_trajectory_.back());
        ADEBUG << "Open space init point added!";
        ptr_debug->mutable_planning_data()->mutable_open_space()->CopyFrom(
            last_open_space_debug_);
        ADEBUG << "Open space debug information added!";
      }

      // trajectory partition and choose the current trajectory to follow
      trajectory_partition_status =
          TrajectoryPartition(last_trajectory_, ptr_trajectory_pb);
      status = Status(ErrorCode::OK,
                      "use last planning result from OpenSpacePlanner");

      if (FLAGS_enable_record_debug && FLAGS_export_chart) {
        FillPlanningPb(current_time_stamp, ptr_trajectory_pb);
        ExportOpenSpaceChart(ptr_debug, ptr_trajectory_pb);
      }
    } else {
      return status;
    }
  } else if (status ==
             Status(ErrorCode::OK, "Vehicle is near to destination")) {
    GenerateStopTrajectory(ptr_trajectory_pb);
  } else {
    return status;
  }

  if (trajectory_partition_status != Status::OK()) {
    return trajectory_partition_status;
  }

  BuildPredictedEnvironment(frame_.get()->obstacles());

  if (!IsCollisionFreeTrajectory(*ptr_trajectory_pb)) {
    return Status(ErrorCode::PLANNING_ERROR, "Collision Check failed");
  }
  return status;
}

Status OpenSpacePlanning::TrajectoryPartition(
    const std::unique_ptr<PublishableTrajectory>& last_publishable_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  std::vector<common::TrajectoryPoint>
      uninterpolated_stitched_trajectory_to_end =
          last_publishable_trajectory->trajectory_points();

  // interpolate the stitched trajectory
  std::vector<common::TrajectoryPoint> stitched_trajectory_to_end;
  size_t interpolated_pieces_num = 50;
  for (size_t i = 0; i < uninterpolated_stitched_trajectory_to_end.size() - 1;
       i++) {
    double relative_time_interval =
        (uninterpolated_stitched_trajectory_to_end[i + 1].relative_time() -
         uninterpolated_stitched_trajectory_to_end[i].relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    stitched_trajectory_to_end.push_back(
        uninterpolated_stitched_trajectory_to_end[i]);
    for (size_t j = 0; j < interpolated_pieces_num - 1; j++) {
      double relative_time =
          uninterpolated_stitched_trajectory_to_end[i].relative_time() +
          (static_cast<double>(j) + 1) * relative_time_interval;
      stitched_trajectory_to_end.emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              uninterpolated_stitched_trajectory_to_end[i],
              uninterpolated_stitched_trajectory_to_end[i + 1], relative_time));
    }
  }
  stitched_trajectory_to_end.push_back(
      uninterpolated_stitched_trajectory_to_end
          [uninterpolated_stitched_trajectory_to_end.size() - 1]);
  double distance_s = 0.0;
  apollo::planning_internal::Trajectories trajectory_partition;
  std::vector<apollo::canbus::Chassis::GearPosition> gear_positions;

  apollo::common::Trajectory* current_trajectory =
      trajectory_partition.add_trajectory();

  // set initial gear position for first ADCTrajectory depending on v
  // and check potential edge cases
  const size_t initial_gear_check_horizon = 3;
  const double kepsilon = 1e-6;
  size_t horizon = stitched_trajectory_to_end.size();
  size_t initial_horizon = std::min(horizon, initial_gear_check_horizon);
  int direction_flag = 0;
  size_t i = 0;
  int j = 0;
  int init_direction = 0;
  while (i != initial_horizon) {
    if (stitched_trajectory_to_end[j].v() > kepsilon) {
      i++;
      j++;
      direction_flag++;
      if (init_direction == 0) {
        init_direction++;
      }
    } else if (stitched_trajectory_to_end[j].v() < -kepsilon) {
      i++;
      j++;
      direction_flag--;
      if (init_direction == 0) {
        init_direction--;
      }
    } else {
      j++;
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
  // start point of gears plot
  // TODO(Jiaxuan): move it out of TrajectoryPartition() and add a flag on it
  plot_gear_shift_.clear();
  plot_gear_shift_time_.clear();
  if (gear_positions.back() == canbus::Chassis::GEAR_DRIVE) {
    plot_gear_shift_.push_back(1.0);
  } else {
    plot_gear_shift_.push_back(-1.0);
  }
  plot_gear_shift_time_.push_back(0.0);

  // partition trajectory points into each trajectory
  for (size_t i = 0; i < horizon; i++) {
    // shift from GEAR_DRIVE to GEAR_REVERSE if v < 0
    // then add a new trajectory with GEAR_REVERSE
    if (stitched_trajectory_to_end[i].v() < -kepsilon &&
        gear_positions.back() == canbus::Chassis::GEAR_DRIVE) {
      current_trajectory = trajectory_partition.add_trajectory();
      gear_positions.push_back(canbus::Chassis::GEAR_REVERSE);
      plot_gear_shift_.push_back(-1.0);
      plot_gear_shift_time_.push_back(
          stitched_trajectory_to_end[i].relative_time());
      distance_s = 0.0;
    }
    // shift from GEAR_REVERSE to GEAR_DRIVE if v > 0
    // then add a new trajectory with GEAR_DRIVE
    if (stitched_trajectory_to_end[i].v() > kepsilon &&
        gear_positions.back() == canbus::Chassis::GEAR_REVERSE) {
      current_trajectory = trajectory_partition.add_trajectory();
      gear_positions.push_back(canbus::Chassis::GEAR_DRIVE);
      plot_gear_shift_.push_back(1.0);
      plot_gear_shift_time_.push_back(
          stitched_trajectory_to_end[i].relative_time());
      distance_s = 0.0;
    }

    auto* point = current_trajectory->add_trajectory_point();
    int gear_drive = 1;
    if (gear_positions.back() == canbus::Chassis::GEAR_REVERSE) gear_drive = -1;
    point->set_relative_time(stitched_trajectory_to_end[i].relative_time());
    point->mutable_path_point()->set_x(
        stitched_trajectory_to_end[i].path_point().x());
    point->mutable_path_point()->set_y(
        stitched_trajectory_to_end[i].path_point().y());
    if (gear_drive == 1) {
      point->mutable_path_point()->set_theta(
          stitched_trajectory_to_end[i].path_point().theta());
    } else {
      point->mutable_path_point()->set_theta(common::math::NormalizeAngle(
          stitched_trajectory_to_end[i].path_point().theta() + M_PI));
    }
    if (i > 0) {
      distance_s +=
          std::sqrt((stitched_trajectory_to_end[i].path_point().x() -
                     stitched_trajectory_to_end[i - 1].path_point().x()) *
                        (stitched_trajectory_to_end[i].path_point().x() -
                         stitched_trajectory_to_end[i - 1].path_point().x()) +
                    (stitched_trajectory_to_end[i].path_point().y() -
                     stitched_trajectory_to_end[i - 1].path_point().y()) *
                        (stitched_trajectory_to_end[i].path_point().y() -
                         stitched_trajectory_to_end[i - 1].path_point().y()));
    }
    point->mutable_path_point()->set_s(distance_s);

    point->set_v(stitched_trajectory_to_end[i].v() * gear_drive);
    // TODO(Jiaxuan): Verify this steering to kappa equation and use parameter
    // from config
    const auto& vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();
    point->mutable_path_point()->set_kappa(
        std::tan(stitched_trajectory_to_end[i].steer()) /
        vehicle_config.vehicle_param().wheel_base() * gear_drive);
    point->set_a(stitched_trajectory_to_end[i].a() * gear_drive);
  }

  // Choose the one to follow based on the closest partitioned trajectory
  size_t trajectories_size = trajectory_partition.trajectory_size();
  size_t current_trajectory_index = 0;
  int closest_trajectory_point_index = 0;
  constexpr double kepsilon_to_destination = 1e-6;
  constexpr double heading_searching_range = 0.3;
  bool flag_change_to_next = false;
  // Could have a big error in vehicle state in single thread mode!!! As the
  // vehicle state is only updated at the every beginning at RunOnce()
  VehicleState vehicle_state = frame_->vehicle_state();

  auto comp = [](const std::pair<std::pair<size_t, int>, double>& left,
                 const std::pair<std::pair<size_t, int>, double>& right) {
    return left.second >= right.second;
  };
  std::priority_queue<std::pair<std::pair<size_t, int>, double>,
                      std::vector<std::pair<std::pair<size_t, int>, double>>,
                      decltype(comp)>
      closest_points(comp);

  for (size_t i = 0; i < trajectories_size; i++) {
    double min_distance = std::numeric_limits<double>::max();
    const apollo::common::Trajectory trajectory =
        trajectory_partition.trajectory(static_cast<int>(i));
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
            heading_searching_range) {
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

    for (int j = 0; j < trajectory_size; j++) {
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
          trajectory_partition
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
          heading_searching_range) {
        current_trajectory_index = closest_point.first.first;
        closest_trajectory_point_index = closest_point.first.second;
        break;
      }
    }
  }

  // reassign relative time and relative s to have the closest point as origin
  // point
  ptr_trajectory_pb->mutable_trajectory_point()->CopyFrom(
      *(trajectory_partition
            .mutable_trajectory(static_cast<int>(current_trajectory_index))
            ->mutable_trajectory_point()));
  double time_shift =
      ptr_trajectory_pb->trajectory_point(closest_trajectory_point_index)
          .relative_time();
  double s_shift =
      ptr_trajectory_pb->trajectory_point(closest_trajectory_point_index)
          .path_point()
          .s();
  int trajectory_size = ptr_trajectory_pb->trajectory_point_size();
  for (int i = 0; i < trajectory_size; i++) {
    apollo::common::TrajectoryPoint* trajectory_point =
        ptr_trajectory_pb->mutable_trajectory_point(i);
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
  ptr_trajectory_pb->set_gear(gear_positions[current_trajectory_index]);
  return Status::OK();
}

void OpenSpacePlanning::AddOpenSpaceTrajectory(
    planning_internal::Debug* debug) {
  auto chart = debug->mutable_planning_data()->add_chart();
  auto open_space_debug = debug->planning_data().open_space();
  chart->set_title("Open Space Trajectory Visualization");
  auto* options = chart->mutable_options();
  CHECK_EQ(open_space_debug.xy_boundary_size(), 4);
  options->mutable_x()->set_min(open_space_debug.xy_boundary(0) - 1.0);
  options->mutable_x()->set_max(open_space_debug.xy_boundary(1) + 1.0);
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_min(open_space_debug.xy_boundary(2) - 1.0);
  options->mutable_y()->set_max(open_space_debug.xy_boundary(3) + 1.0);
  options->mutable_y()->set_label_string("y (meter)");
  int obstacle_index = 1;
  for (const auto& obstacle : open_space_debug.obstacles()) {
    auto* obstacle_outline = chart->add_line();
    obstacle_outline->set_label("boundary_" + std::to_string(obstacle_index));
    obstacle_index += 1;
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); vertice_index++) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }
    // Set chartJS's dataset properties
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    (*obstacle_properties)["borderWidth"] = "2";
    (*obstacle_properties)["pointRadius"] = "0";
    (*obstacle_properties)["lineTension"] = "0";
    (*obstacle_properties)["fill"] = "false";
    (*obstacle_properties)["showLine"] = "true";
  }

  auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* smoothed_line = chart->add_line();
  smoothed_line->set_label("smoothed");
  for (const auto& point : smoothed_trajectory.vehicle_motion_point()) {
    auto* point_debug = smoothed_line->add_point();
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  // Set chartJS's dataset properties
  auto* smoothed_properties = smoothed_line->mutable_properties();
  (*smoothed_properties)["borderWidth"] = "2";
  (*smoothed_properties)["pointRadius"] = "0";
  (*smoothed_properties)["lineTension"] = "0";
  (*smoothed_properties)["fill"] = "false";
  (*smoothed_properties)["showLine"] = "true";

  auto warm_start_trajectory = open_space_debug.warm_start_trajectory();
  auto* warm_start_line = chart->add_line();
  warm_start_line->set_label("warm_start");
  for (const auto& point : warm_start_trajectory.vehicle_motion_point()) {
    auto* point_debug = warm_start_line->add_point();
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  // Set chartJS's dataset properties
  auto* warm_start_properties = warm_start_line->mutable_properties();
  (*warm_start_properties)["borderWidth"] = "2";
  (*warm_start_properties)["pointRadius"] = "0";
  (*warm_start_properties)["lineTension"] = "0";
  (*warm_start_properties)["fill"] = "false";
  (*warm_start_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddStitchSpeedProfile(planning_internal::Debug* debug) {
  auto chart = debug->mutable_planning_data()->add_chart();
  auto open_space_debug = debug->planning_data().open_space();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  for (const auto& point : last_trajectory_->trajectory_points()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() + last_trajectory_->header_time());
    point_debug->set_y(point.v());
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  /*
  auto* gear_shift_line = chart->add_line();
  gear_shift_line->set_label("Gear");
  size_t gear_shift_num =
      std::min(plot_gear_shift_time_.size(), plot_gear_shift_.size());
  for (unsigned int gear_index = 0; gear_index < gear_shift_num; gear_index++) {
    if (gear_index > 0) {
      auto* point_debug = gear_shift_line->add_point();
      point_debug->set_x(plot_gear_shift_time_[gear_index]);
      point_debug->set_y(plot_gear_shift_[gear_index - 1]);
    }
    auto* point_debug = gear_shift_line->add_point();
    point_debug->set_x(plot_gear_shift_time_[gear_index]);
    point_debug->set_y(plot_gear_shift_[gear_index]);
  }
  // Set chartJS's dataset properties
  auto* gear_line_properties = gear_shift_line->mutable_properties();
  (*gear_line_properties)["borderWidth"] = "2";
  (*gear_line_properties)["pointRadius"] = "0";
  (*gear_line_properties)["lineTension"] = "0";
  (*gear_line_properties)["fill"] = "false";
  (*gear_line_properties)["showLine"] = "true";
  */
}

void OpenSpacePlanning::AddPublishedSpeed(
    planning_internal::Debug* debug, ADCTrajectory* const ptr_trajectory_pb) {
  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(10.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("speed (m/s)");

  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  for (const auto& point : ptr_trajectory_pb->trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       ptr_trajectory_pb->header().timestamp_sec());
    if (ptr_trajectory_pb->gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.v());
    if (ptr_trajectory_pb->gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.v());
  }
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Current Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

void OpenSpacePlanning::AddPublishedAcceleration(
    planning_internal::Debug* debug, ADCTrajectory* const ptr_trajectory_pb) {
  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  options->mutable_x()->set_window_size(10.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(2.1);
  options->mutable_y()->set_max(-1.1);
  options->mutable_y()->set_label_string("Acceleration (m/s^2)");

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : ptr_trajectory_pb->trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       ptr_trajectory_pb->header().timestamp_sec());
    if (ptr_trajectory_pb->gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.a());
    if (ptr_trajectory_pb->gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.a());
  }
  // Set chartJS's dataset properties
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Current Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

void OpenSpacePlanning::ExportOpenSpaceChart(
    planning_internal::Debug* debug, ADCTrajectory* const ptr_trajectory_pb) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_record_debug) {
    AddOpenSpaceTrajectory(debug);
    AddStitchSpeedProfile(debug);
    AddPublishedSpeed(debug, ptr_trajectory_pb);
    AddPublishedAcceleration(debug, ptr_trajectory_pb);
  }
}

bool OpenSpacePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  // TODO(All): check config params
  return true;
}

void OpenSpacePlanning::FillPlanningPb(const double timestamp,
                                       ADCTrajectory* const ptr_trajectory_pb) {
  ptr_trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (!local_view_.prediction_obstacles->has_header()) {
    ptr_trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    ptr_trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    ptr_trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }
  ptr_trajectory_pb->mutable_routing_header()->CopyFrom(
      local_view_.routing->header());

  if (FLAGS_use_planning_fallback &&
      ptr_trajectory_pb->trajectory_point_size() == 0) {
    SetFallbackTrajectory(ptr_trajectory_pb);
  }
  const double dt = timestamp - Clock::NowInSeconds();
  for (auto& p : *ptr_trajectory_pb->mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() - dt);
  }
}

bool OpenSpacePlanning::IsCollisionFreeTrajectory(
    const ADCTrajectory& trajectory_pb) {
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();
  int point_size = trajectory_pb.trajectory_point().size();
  for (int i = 0; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.trajectory_point(i);
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    size_t predicted_time_horizon = predicted_bounding_rectangles_.size();
    for (size_t j = 0; j < predicted_time_horizon; j++) {
      for (const auto& obstacle_box : predicted_bounding_rectangles_[j]) {
        if (ego_box.HasOverlap(obstacle_box)) {
          return false;
        }
      }
    }
  }
  return true;
}

void OpenSpacePlanning::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles) {
  predicted_bounding_rectangles_.clear();
  double relative_time = 0.0;
  while (relative_time < FLAGS_open_space_prediction_time_horizon) {
    std::vector<Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles) {
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      predicted_env.push_back(std::move(box));
    }
    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

apollo::common::Status OpenSpacePlanning::GenerateGearShiftTrajectory(
    const std::vector<double>& end_pose,
    const apollo::canbus::Chassis::GearPosition& gear_position,
    ADCTrajectory* trajectory_pb) {
  if (end_pose.size() != 4) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "End pose not valid in GenerateGearShiftTrajectory!");
  }

  trajectory_pb->clear_trajectory_point();

  trajectory_pb->set_gear(gear_position);

  // TODO(QiL): move this to config after finalize the logic
  const double max_t = 3.0;
  const double unit_t = 0.02;

  TrajectoryPoint tp;
  auto path_point = tp.mutable_path_point();
  path_point->set_x(end_pose[0]);
  path_point->set_y(end_pose[1]);
  path_point->set_theta(end_pose[2]);
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);
    auto next_point = trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }

  return Status::OK();
}

void OpenSpacePlanning::GenerateStopTrajectory(
    ADCTrajectory* const ptr_trajectory_pb) {
  double relative_time = 0.0;
  constexpr int stop_trajectory_length = 10;
  constexpr double relative_stop_time = 0.1;
  for (size_t i = 0; i < stop_trajectory_length; i++) {
    auto* point = ptr_trajectory_pb->add_trajectory_point();
    point->mutable_path_point()->set_x(frame_->vehicle_state().x());
    point->mutable_path_point()->set_y(frame_->vehicle_state().x());
    point->mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point->mutable_path_point()->set_s(0.0);
    point->mutable_path_point()->set_kappa(0.0);
    point->set_relative_time(relative_time);
    point->set_v(0.0);
    point->set_a(0.0);
    relative_time += relative_stop_time;
  }
}

}  // namespace planning
}  // namespace apollo
