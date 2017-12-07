/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning.h"

#include <algorithm>
#include <vector>

#include "google/protobuf/repeated_field.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::common::VehicleState;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;

std::string Planning::Name() const { return "planning"; }

void Planning::RegisterPlanners() {
  planner_factory_.Register(
      PlanningConfig::RTK, []() -> Planner* { return new RTKReplayPlanner(); });
  planner_factory_.Register(PlanningConfig::EM,
                            []() -> Planner* { return new EMPlanner(); });
}

Status Planning::InitFrame(const uint32_t sequence_num,
                           const TrajectoryPoint& planning_start_point,
                           const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, planning_start_point, vehicle_state));
  if (FLAGS_enable_prediction && !AdapterManager::GetPrediction()->Empty()) {
    const auto& prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();
    frame_->SetPrediction(prediction);
    ADEBUG << "Get prediction: " << prediction.DebugString();
  }
  auto status = frame_->Init();
  if (!status.ok()) {
    AERROR << "failed to init frame";
    return Status(ErrorCode::PLANNING_ERROR, "init frame failed");
  }
  return Status::OK();
}

Status Planning::Init() {
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                               &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_planning_adapter_config_filename);
  }
  if (AdapterManager::GetLocalization() == nullptr) {
    std::string error_msg("Localization is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (AdapterManager::GetChassis() == nullptr) {
    std::string error_msg("Chassis is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (AdapterManager::GetRoutingResponse() == nullptr) {
    std::string error_msg("RoutingResponse is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (AdapterManager::GetRoutingRequest() == nullptr) {
    std::string error_msg("RoutingRequest is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() == nullptr) {
    std::string error_msg("Prediction is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (FLAGS_enable_traffic_light &&
      AdapterManager::GetTrafficLightDetection() == nullptr) {
    std::string error_msg("Traffic Light Detection is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  ReferenceLineProvider::instance()->Init(
      hdmap_, config_.qp_spline_reference_line_smoother_config());

  RegisterPlanners();
  planner_ = planner_factory_.CreateObject(config_.planner_type());
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  return planner_->Init(config_);
}

bool Planning::IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

Status Planning::Start() {
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_planning_loop_rate), &Planning::OnTimer, this);
  ReferenceLineProvider::instance()->Start();
  AINFO << "Planning started";
  return Status::OK();
}

void Planning::OnTimer(const ros::TimerEvent&) { RunOnce(); }

void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb,
                                 double timestamp) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
  if (AdapterManager::GetRoutingResponse() &&
      !AdapterManager::GetRoutingResponse()->Empty()) {
    trajectory_pb->mutable_routing_header()->CopyFrom(
        AdapterManager::GetRoutingResponse()->GetLatestObserved().header());
  }

  // NOTICE:
  // Since we are using the time at each cycle beginning as timestamp, the
  // relative time of each trajectory point should be modified so that we can
  // use the current timestamp in header.

  // auto* trajectory_points = trajectory_pb.mutable_trajectory_point();
  if (!FLAGS_planning_test_mode) {
    const double dt = timestamp - Clock::NowInSeconds();
    for (auto& p : *trajectory_pb->mutable_trajectory_point()) {
      p.set_relative_time(p.relative_time() + dt);
    }
  }
  Publish(trajectory_pb);
}

void Planning::RunOnce() {
  const double start_timestamp = Clock::NowInSeconds();

  // snapshot all coming data
  AdapterManager::Observe();

  ADCTrajectory not_ready_pb;
  auto* not_ready = not_ready_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();
  if (AdapterManager::GetLocalization()->Empty()) {
    not_ready->set_reason("localization not ready");
  } else if (AdapterManager::GetChassis()->Empty()) {
    not_ready->set_reason("chassis not ready");
  } else if (AdapterManager::GetRoutingResponse()->Empty()) {
    not_ready->set_reason("routing not ready");
  }
  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  // localization
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();

  // chassis
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();

  Status status =
      VehicleStateProvider::instance()->Update(localization, chassis);
  VehicleState vehicle_state =
      VehicleStateProvider::instance()->vehicle_state();

  // estimate (x, y) at current timestamp
  // This estimate is only valid if the current time and vehicle state timestamp
  // differs only a small amount (20ms). When the different is too large, the
  // estimation is invalid.
  DCHECK_GE(start_timestamp, vehicle_state.timestamp());
  if (FLAGS_estimate_current_vehicle_state &&
      start_timestamp - vehicle_state.timestamp() < 0.020) {
    auto future_xy = VehicleStateProvider::instance()->EstimateFuturePosition(
        start_timestamp - vehicle_state.timestamp());
    vehicle_state.set_x(future_xy.x());
    vehicle_state.set_y(future_xy.y());
    vehicle_state.set_timestamp(start_timestamp);
  }

  if (!status.ok() || !IsVehicleStateValid(vehicle_state)) {
    AERROR << "Update VehicleStateProvider failed.";
    not_ready->set_reason("Update VehicleStateProvider failed.");
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  // Update reference line provider
  ReferenceLineProvider::instance()->UpdateVehicleState(vehicle_state);

  if (!ReferenceLineProvider::instance()->UpdateRoutingResponse(
          AdapterManager::GetRoutingResponse()->GetLatestObserved())) {
    AERROR << "Failed to update routing in reference line provider";
    return;
  }

  if (FLAGS_enable_prediction && AdapterManager::GetPrediction()->Empty()) {
    not_ready->set_reason("prediction not ready");
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  bool is_replan = false;
  const auto stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          vehicle_state, start_timestamp, planning_cycle_time,
          last_publishable_trajectory_.get(), &is_replan);

  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);
  ADCTrajectory trajectory_pb;
  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(trajectory_pb.mutable_debug());
  }
  trajectory_pb.mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);
  if (!status.ok()) {
    AERROR << "Init frame failed";
    if (FLAGS_publish_estop) {
      ADCTrajectory estop;
      estop.mutable_estop();
      status.Save(estop.mutable_header()->mutable_status());
      PublishPlanningPb(&estop, start_timestamp);
    }
    if (frame_) {
      auto seq_num = frame_->SequenceNum();
      FrameHistory::instance()->Add(seq_num, std::move(frame_));
    }
    return;
  }

  status = Plan(start_timestamp, stitching_trajectory, &trajectory_pb);

  const auto time_diff_ms = (Clock::NowInSeconds() - start_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  trajectory_pb.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: " << trajectory_pb.latency_stats().DebugString();

  auto* ref_line_task = trajectory_pb.mutable_latency_stats()->add_task_stats();
  ref_line_task->set_time_ms(
      ReferenceLineProvider::instance()->LastTimeDelay() * 1000.0);
  ref_line_task->set_name("ReferenceLineProvider");

  if (!status.ok()) {
    status.Save(trajectory_pb.mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      trajectory_pb.mutable_estop();
    }
  }

  trajectory_pb.set_is_replan(is_replan);
  PublishPlanningPb(&trajectory_pb, start_timestamp);
  ADEBUG << "Planning pb:" << trajectory_pb.header().DebugString();

  if (frame_) {
    auto seq_num = frame_->SequenceNum();
    FrameHistory::instance()->Add(seq_num, std::move(frame_));
  }
}

void Planning::Stop() {
  AERROR << "Planning Stop is called";
  ReferenceLineProvider::instance()->Stop();
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
}

void Planning::SetLastPublishableTrajectory(
    const ADCTrajectory& adc_trajectory) {
  last_publishable_trajectory_.reset(new PublishableTrajectory(adc_trajectory));
}

Status Planning::Plan(const double current_time_stamp,
                      const std::vector<TrajectoryPoint>& stitching_trajectory,
                      ADCTrajectory* trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
  }
  auto status = Status::OK();
  for (auto& reference_line_info : frame_->reference_line_info()) {
    status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                            &reference_line_info);
    if (status == Status::OK()) {
      if (FLAGS_prioritize_change_lane && reference_line_info.IsDrivable() &&
          reference_line_info.IsChangeLanePath()) {
        ADEBUG << "Found change lane line, skip other reference line";
        break;
      }
    } else {
      AERROR << "planner failed to make a driving plan for: "
             << reference_line_info.Lanes().Id();
    }
  }
  const auto* best_reference_line = frame_->FindDriveReferenceLineInfo();
  if (!best_reference_line) {
    std::string msg(
        "planner failed to make a driving plan because NO valid reference line "
        "info.");
    AERROR << msg;
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ptr_debug->MergeFrom(best_reference_line->debug());
  trajectory_pb->mutable_latency_stats()->MergeFrom(
      best_reference_line->latency_stats());

  best_reference_line->ExportDecision(trajectory_pb->mutable_decision());

  // Add debug information.
  if (FLAGS_enable_record_debug) {
    auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
    reference_line->set_name("planning_reference_line");
    const auto& reference_points =
        best_reference_line->reference_line().reference_points();
    for (const auto& reference_point : reference_points) {
      auto* path_point = reference_line->add_path_point();
      path_point->set_x(reference_point.x());
      path_point->set_y(reference_point.y());
      path_point->set_theta(reference_point.heading());
      path_point->set_kappa(reference_point.kappa());
      path_point->set_dkappa(reference_point.dkappa());
    }
  }

  last_publishable_trajectory_.reset(new PublishableTrajectory(
      current_time_stamp, best_reference_line->trajectory()));

  ADEBUG << "current_time_stamp: " << std::to_string(current_time_stamp);

  last_publishable_trajectory_->PrependTrajectoryPoints(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1);

  for (size_t i = 0; i < last_publishable_trajectory_->NumOfPoints(); ++i) {
    if (last_publishable_trajectory_->TrajectoryPointAt(i).relative_time() >
        FLAGS_trajectory_time_high_density_period) {
      break;
    }
    ADEBUG << last_publishable_trajectory_->TrajectoryPointAt(i)
                  .ShortDebugString();
  }

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  return status;
}

}  // namespace planning
}  // namespace apollo
