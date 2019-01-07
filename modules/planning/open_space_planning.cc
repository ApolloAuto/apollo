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
#include <queue>
#include <utility>

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
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
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

  trajectory_partitioner_.reset(new TrajectoryPartitioner());
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
          std::vector<TrajectoryPoint>(last_stitching_trajectory_.begin(),
                                       last_stitching_trajectory_.end() - 1));
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
    trajectory_partition_status = trajectory_partitioner_->TrajectoryPartition(
        last_publishable_trajectory_, frame_.get(), ptr_trajectory_pb);

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
          trajectory_partitioner_->TrajectoryPartition(
              last_trajectory_, frame_.get(), ptr_trajectory_pb);
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
  for (const auto& point : *last_trajectory_) {
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
  ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
}

}  // namespace planning
}  // namespace apollo
