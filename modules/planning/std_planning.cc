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

#include "modules/planning/std_planning.h"

#include <list>
#include <utility>

#include "gtest/gtest_prod.h"

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/traffic_rules/traffic_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
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

StdPlanning::~StdPlanning() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  FrameHistory::Instance()->Clear();
  PlanningContext::MutablePlanningStatus()->Clear();
  last_routing_.Clear();
  EgoInfo::Instance()->Clear();
}

std::string StdPlanning::Name() const { return "std_planning"; }

Status StdPlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);

  planner_dispatcher_->Init();

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;

  // clear planning status
  PlanningContext::MutablePlanningStatus()->Clear();

  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map";

  // instantiate reference line provider
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
  reference_line_provider_->Start();

  // dispatch planner
  planner_ = planner_dispatcher_->DispatchPlanner();
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  start_time_ = Clock::NowInSeconds();
  return planner_->Init(config_);
}

Status StdPlanning::InitFrame(const uint32_t sequence_num,
                              const TrajectoryPoint& planning_start_point,
                              const double start_time,
                              const VehicleState& vehicle_state,
                              ADCTrajectory* output_trajectory) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         start_time, vehicle_state,
                         reference_line_provider_.get(), output_trajectory));
  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
    std::string msg = "Failed to create reference line";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  DCHECK_EQ(reference_lines.size(), segments.size());

  auto forword_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

  for (auto& ref_line : reference_lines) {
    if (!ref_line.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                         FLAGS_look_backward_distance, forword_limit)) {
      std::string msg = "Fail to shrink reference line.";
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  for (auto& seg : segments) {
    if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                    FLAGS_look_backward_distance, forword_limit)) {
      std::string msg = "Fail to shrink routing segments.";
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  auto status = frame_->Init(reference_lines, segments,
                             reference_line_provider_->FutureRouteWaypoints());
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

void StdPlanning::GenerateStopTrajectory(ADCTrajectory* trajectory_pb) {
  trajectory_pb->clear_trajectory_point();

  const auto& vehicle_state = VehicleStateProvider::Instance()->vehicle_state();
  const double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;

  TrajectoryPoint tp;
  auto path_point = tp.mutable_path_point();
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);
    auto next_point = trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}

void StdPlanning::RunOnce(const LocalView& local_view,
                          ADCTrajectory* const trajectory_pb) {
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

  auto* not_ready = trajectory_pb->mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (!status.ok() || !IsVehicleStateValid(vehicle_state)) {
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    FillPlanningPb(start_timestamp, trajectory_pb);
    GenerateStopTrajectory(trajectory_pb);
    return;
  }

  if (IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    PlanningContext::MutablePlanningStatus()->Clear();
    reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
  }

  // Update reference line provider and reset pull over if necessary
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  // planning is triggered by prediction data, but we can still use an estimated
  // cycle time for stitching
  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  std::vector<TrajectoryPoint> stitching_trajectory;
  std::string replan_reason;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get(), &replan_reason);

  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  bool update_ego_info =
      EgoInfo::Instance()->Update(stitching_trajectory.back(), vehicle_state);
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
                     vehicle_state, trajectory_pb);

  if (update_ego_info && status.ok()) {
    EgoInfo::Instance()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
  }

  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(trajectory_pb->mutable_debug());
  }
  trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);
  if (!status.ok() || !update_ego_info) {
    AERROR << status.ToString();
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
      trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(trajectory_pb->mutable_header()->mutable_status());
      GenerateStopTrajectory(trajectory_pb);
    }

    FillPlanningPb(start_timestamp, trajectory_pb);
    frame_->mutable_trajectory()->CopyFrom(*trajectory_pb);
    const uint32_t n = frame_->SequenceNum();
    FrameHistory::Instance()->Add(n, std::move(frame_));
    return;
  }

  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
      continue;
    }
  }

  status = Plan(start_timestamp, stitching_trajectory, trajectory_pb);

  for (const auto& p : trajectory_pb->trajectory_point()) {
    ADEBUG << p.DebugString();
  }
  const auto time_diff_ms = (Clock::NowInSeconds() - start_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << trajectory_pb->latency_stats().DebugString();

  auto* ref_line_task =
      trajectory_pb->mutable_latency_stats()->add_task_stats();
  ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                             1000.0);
  ref_line_task->set_name("ReferenceLineProvider");

  if (!status.ok()) {
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

  trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (trajectory_pb->is_replan()) {
    trajectory_pb->set_replan_reason(replan_reason);
  }

  FillPlanningPb(start_timestamp, trajectory_pb);
  ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();

  frame_->mutable_trajectory()->CopyFrom(*trajectory_pb);
  if (FLAGS_enable_planning_smoother) {
    planning_smoother_.Smooth(FrameHistory::Instance(), frame_.get(),
                              trajectory_pb);
  }

  const uint32_t n = frame_->SequenceNum();
  FrameHistory::Instance()->Add(n, std::move(frame_));
}

void StdPlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);
  }
}

Status StdPlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get());

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      EgoInfo::Instance()->front_clear_distance());

  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (!best_ref_info) {
    std::string msg("planner failed to make a driving plan");
    AERROR << msg;
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (FLAGS_export_chart) {
    ExportChart(best_ref_info->debug(), ptr_debug);
  } else {
    ptr_debug->MergeFrom(best_ref_info->debug());
    ExportReferenceLineDebug(ptr_debug);
  }
  trajectory_pb->mutable_latency_stats()->MergeFrom(
      best_ref_info->latency_stats());
  // set right of way status
  trajectory_pb->set_right_of_way_status(best_ref_info->GetRightOfWayStatus());
  for (const auto& id : best_ref_info->TargetLaneId()) {
    trajectory_pb->add_lane_id()->CopyFrom(id);
  }

  trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

  if (FLAGS_enable_rss_info) {
    trajectory_pb->mutable_rss_info()->CopyFrom(best_ref_info->rss_info());
  }

  best_ref_info->ExportDecision(trajectory_pb->mutable_decision());

  // Add debug information.
  if (FLAGS_enable_record_debug) {
    auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
    reference_line->set_name("planning_reference_line");
    const auto& reference_points =
        best_ref_info->reference_line().reference_points();
    double s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool empty_path = true;
    for (const auto& reference_point : reference_points) {
      auto* path_point = reference_line->add_path_point();
      path_point->set_x(reference_point.x());
      path_point->set_y(reference_point.y());
      path_point->set_theta(reference_point.heading());
      path_point->set_kappa(reference_point.kappa());
      path_point->set_dkappa(reference_point.dkappa());
      if (empty_path) {
        path_point->set_s(0.0);
        empty_path = false;
      } else {
        double dx = reference_point.x() - prev_x;
        double dy = reference_point.y() - prev_y;
        s += std::hypot(dx, dy);
        path_point->set_s(s);
      }
      prev_x = reference_point.x();
      prev_y = reference_point.y();
    }
  }

  last_publishable_trajectory_.reset(new PublishableTrajectory(
      current_time_stamp, best_ref_info->trajectory()));

  ADEBUG << "current_time_stamp: " << std::to_string(current_time_stamp);

  if (FLAGS_enable_stitch_last_trajectory) {
    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));
  }

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  best_ref_info->ExportEngageAdvice(trajectory_pb->mutable_engage_advice());

  return status;
}

bool StdPlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_standard_planning_config()) {
    return false;
  }
  if (config.standard_planning_config()
          .planner_public_road_config()
          .scenario_type_size() == 0) {
    return false;
  }
  // TODO(All): check other config params
  return true;
}

}  // namespace planning
}  // namespace apollo
