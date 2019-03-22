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

#include <list>
#include <utility>

#include "cyber/common/file.h"
#include "gtest/gtest_prod.h"

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/task_factory.h"
#include "modules/planning/traffic_rules/traffic_decider.h"
#include "modules/planning/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::EngageAdvice;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::dreamview::Chart;
using apollo::hdmap::HDMapUtil;
using apollo::planning_internal::SLFrameDebug;
using apollo::planning_internal::SpeedPlan;
using apollo::planning_internal::STGraphDebug;
using apollo::routing::RoutingResponse;

OnLanePlanning::~OnLanePlanning() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  FrameHistory::Instance()->Clear();
  PlanningContext::MutablePlanningStatus()->Clear();
  last_routing_.Clear();
  EgoInfo::Instance()->Clear();
}

std::string OnLanePlanning::Name() const { return "on_lane_planning"; }

Status OnLanePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);

  planner_dispatcher_->Init();

  CHECK(apollo::cyber::common::GetProtoFromFile(
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

Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));

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

// TODO(all): fix this! this will cause unexpected behavior from controller
void OnLanePlanning::GenerateStopTrajectory(ADCTrajectory* trajectory_pb) {
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

void OnLanePlanning::RunOnce(const LocalView& local_view,
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
    // TODO(all): integrate reverse gear
    trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
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
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);

  std::vector<TrajectoryPoint> stitching_trajectory;
  std::string replan_reason;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get(), &replan_reason);

  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  bool update_ego_info =
      EgoInfo::Instance()->Update(stitching_trajectory.back(), vehicle_state);
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

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
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
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
    // TODO(all): integrate reverse gear
    trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, trajectory_pb);
    frame_->set_current_frame_planned_trajectory(*trajectory_pb);
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

  if (!status.ok()) {
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
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

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    FillPlanningPb(start_timestamp, trajectory_pb);
    ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();
    frame_->set_current_frame_planned_trajectory(*trajectory_pb);
  } else {
    auto* ref_line_task =
        trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    // TODO(all): integrate reverse gear
    trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, trajectory_pb);
    ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();

    frame_->set_current_frame_planned_trajectory(*trajectory_pb);
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(FrameHistory::Instance(), frame_.get(),
                                trajectory_pb);
    }
  }

  const uint32_t n = frame_->SequenceNum();
  FrameHistory::Instance()->Add(n, std::move(frame_));
}

void OnLanePlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
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

Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }

  auto status =
      planner_->Plan(stitching_trajectory.back(), frame_.get(), trajectory_pb);

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      EgoInfo::Instance()->front_clear_distance());

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    const auto& publishable_trajectory =
        frame_->open_space_info().publishable_trajectory_data().first;
    const auto& publishable_trajectory_gear =
        frame_->open_space_info().publishable_trajectory_data().second;
    publishable_trajectory.PopulateTrajectoryProtobuf(trajectory_pb);
    trajectory_pb->set_gear(publishable_trajectory_gear);

    // TODO(QiL): refine engage advice in open space trajectory optimizer.
    auto* engage_advice = trajectory_pb->mutable_engage_advice();
    engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
    engage_advice->set_reason("Keep enage while in parking");

    // TODO(QiL): refine the export decision in open space info
    trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);

    if (FLAGS_enable_record_debug) {
      ptr_debug->MergeFrom(frame_->open_space_info().debug_instance());
      ADEBUG << "Open space debug information added!";
      // call open space info load debug
      // to-do: runxin, create a new flag to enable openspace chart
      ExportOpenSpaceChart(frame_->open_space_info().debug_instance(),
                           ptr_debug);
    }
  } else {
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
      ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
    } else {
      ptr_debug->MergeFrom(best_ref_info->debug());
      ExportReferenceLineDebug(ptr_debug);
    }
    trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_ref_info->latency_stats());
    // set right of way status
    trajectory_pb->set_right_of_way_status(
        best_ref_info->GetRightOfWayStatus());
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

    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));

    last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

    best_ref_info->ExportEngageAdvice(trajectory_pb->mutable_engage_advice());
  }

  return status;
}

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
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

void PopulateChartOptions(double x_min, double x_max, std::string x_label,
                          double y_min, double y_max, std::string y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

void AddStGraph(const STGraphDebug& st_graph, Chart* chart) {
  chart->set_title(st_graph.name());
  PopulateChartOptions(-2.0, 10.0, "t (second)", 0.0, 80.0, "s (meter)", true,
                       chart);

  for (const auto& boundary : st_graph.boundary()) {
    auto* boundary_chart = chart->add_polygon();

    // from 'ST_BOUNDARY_TYPE_' to the end
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);
    boundary_chart->set_label(boundary.name() + "_" + type);
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }
}

void AddSlFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);
  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_x(sl_point.l());
  }
}

void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    Chart* chart) {
  chart->set_title("Speed Plan");
  PopulateChartOptions(0.0, 80.0, "s (meter)", 0.0, 50.0, "v (m/s)", false,
                       chart);

  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
    }

    // Set chartJS's dataset properties
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
    if (speed_plan.name() == "DpStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() == "QpSplineStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    }
  }
}

void OnLanePlanning::ExportOnLaneChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  for (const auto& st_graph : debug_info.planning_data().st_graph()) {
    AddStGraph(st_graph, debug_chart->mutable_planning_data()->add_chart());
  }
  for (const auto& sl_frame : debug_info.planning_data().sl_frame()) {
    AddSlFrame(sl_frame, debug_chart->mutable_planning_data()->add_chart());
  }

  AddSpeedPlan(debug_info.planning_data().speed_plan(),
               debug_chart->mutable_planning_data()->add_chart());
}

void OnLanePlanning::ExportOpenSpaceChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_record_debug) {
    AddOpenSpaceOptimizerResult(debug_info, debug_chart);
    // AddStitchSpeedProfile(debug);
    // AddPublishedSpeed(debug, ptr_trajectory_pb);
    // AddPublishedAcceleration(debug, ptr_trajectory_pb);
  }
}

void OnLanePlanning::AddOpenSpaceOptimizerResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  auto open_space_debug = debug_info.planning_data().open_space();

  chart->set_title("Open Space Trajectory Visualization");
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)",
                       false, chart);

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

}  // namespace planning
}  // namespace apollo
