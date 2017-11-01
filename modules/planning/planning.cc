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
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/trajectory_stitcher/trajectory_stitcher.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
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

Status Planning::InitFrame(const uint32_t sequence_num, const double timestamp,
                           const TrajectoryPoint& init_adc_point) {
  frame_.reset(new Frame(sequence_num));
  frame_->SetPlanningStartPoint(init_adc_point);

  if (AdapterManager::GetRoutingResponse()->Empty()) {
    AERROR << "Routing is empty";
    return Status(ErrorCode::PLANNING_ERROR, "routing is empty");
  }
  frame_->UpdateRoutingResponse(
      AdapterManager::GetRoutingResponse()->GetLatestObserved());
  frame_->SetVehicleInitPose(VehicleState::instance()->pose());

  if (FLAGS_enable_prediction && !AdapterManager::GetPrediction()->Empty()) {
    const auto& prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();
    frame_->SetPrediction(prediction);
    ADEBUG << "Get prediction: " << prediction.DebugString();
  }

  auto status = frame_->Init(config_, timestamp);
  if (!status.ok()) {
    AERROR << "failed to init frame";
    return Status(ErrorCode::PLANNING_ERROR, "init frame failed");
  }
  return Status::OK();
}

bool Planning::HasSignalLight(const PlanningConfig& config) {
  for (const auto& rule_config : config.rule_config()) {
    if (rule_config.rule_id() == RuleConfig::SIGNAL_LIGHT) {
      return true;
    }
  }
  return false;
}

Status Planning::Init() {
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();
  Frame::SetMap(hdmap_);

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
  if (HasSignalLight(config_) &&
      AdapterManager::GetTrafficLightDetection() == nullptr) {
    std::string error_msg("Traffic Light Detection is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->Init(
        hdmap_, config_.qp_spline_reference_line_smoother_config());
  }

  RegisterPlanners();
  planner_ = planner_factory_.CreateObject(config_.planner_type());
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  return planner_->Init(config_);
}

bool Planning::IsVehicleStateValid(const common::VehicleState& vehicle_state) {
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
  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->Start();
  }
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_planning_loop_rate), &Planning::OnTimer, this);
  return Status::OK();
}

void Planning::OnTimer(const ros::TimerEvent&) { RunOnce(); }

void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb,
                                 double timestamp) {
  AdapterManager::FillPlanningHeader(Name(), trajectory_pb);
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

  if (frame_) {
    trajectory_pb->mutable_routing_header()->CopyFrom(
        frame_->routing_response().header());
  }
  AdapterManager::PublishPlanning(*trajectory_pb);
}

void Planning::RunOnce() {
  const double start_timestamp = Clock::NowInSecond();
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
  } else if (FLAGS_enable_prediction &&
             AdapterManager::GetPrediction()->Empty()) {
    not_ready->set_reason("prediction not ready");
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

  common::Status status =
      common::VehicleState::instance()->Update(localization, chassis);
  DCHECK(IsVehicleStateValid(*common::VehicleState::instance()));
  if (!status.ok()) {
    AERROR << "Update VehicleState failed.";
    not_ready->set_reason("Update VehicleState failed.");
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }
  // if reference line is not ready, continue;
  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->UpdateRoutingResponse(
        AdapterManager::GetRoutingResponse()->GetLatestObserved());
    ReferenceLineProvider::instance()->UpdateVehicleStatus(
        common::VehicleState::instance()->pose().position(),
        common::VehicleState::instance()->linear_velocity());
    if (!ReferenceLineProvider::instance()->HasReferenceLine()) {
      not_ready->set_reason("reference line not ready");
      AERROR << not_ready->reason() << "; skip the planning cycle.";
      PublishPlanningPb(&not_ready_pb, start_timestamp);
      return;
    }
  }

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  bool is_auto_mode = chassis.driving_mode() == chassis.COMPLETE_AUTO_DRIVE;
  bool is_replan = false;
  const auto& stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          is_auto_mode, start_timestamp, planning_cycle_time,
          last_publishable_trajectory_.get(), &is_replan);

  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, start_timestamp, stitching_trajectory.back());
  ADCTrajectory trajectory_pb;
  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(trajectory_pb.mutable_debug());
  }
  trajectory_pb.mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSecond() - start_timestamp);
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

  const auto time_diff_ms = (Clock::NowInSecond() - start_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  trajectory_pb.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: " << trajectory_pb.latency_stats().DebugString();

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
  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->Stop();
  }
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
}

void Planning::SetLastPublishableTrajectory(
    const ADCTrajectory& adc_trajectory) {
  last_publishable_trajectory_.reset(new PublishableTrajectory(adc_trajectory));
}

common::Status Planning::Plan(
    const double current_time_stamp,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
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
    AERROR_IF(!status.ok()) << "planner failed to make a driving plan for: "
                            << reference_line_info.Lanes().Id();
  }

  const auto* best_reference_line = frame_->FindDriveReferenceLineInfo();
  if (!best_reference_line) {
    std::string msg(
        "planner failed to make a driving plan because NO "
        "best_reference_line "
        "can be provided.");
    AERROR << msg;
    last_publishable_trajectory_->Clear();
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

  last_publishable_trajectory_->PrependTrajectoryPoints(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1);

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  return status;
}

}  // namespace planning
}  // namespace apollo
