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

Status Planning::InitFrame(const uint32_t sequence_num, const double time_stamp,
                           const TrajectoryPoint& init_adc_point) {
  frame_.reset(new Frame(sequence_num));
  frame_->SetPlanningStartPoint(init_adc_point);

  if (AdapterManager::GetRoutingResponse()->Empty()) {
    AERROR << "Routing is empty";
    return Status(ErrorCode::PLANNING_ERROR, "routing is empty");
  }
  frame_->SetVehicleInitPose(VehicleState::instance()->pose());
  frame_->SetRoutingResponse(
      AdapterManager::GetRoutingResponse()->GetLatestObserved());
  if (FLAGS_enable_prediction && !AdapterManager::GetPrediction()->Empty()) {
    const auto& prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();
    frame_->SetPrediction(prediction);
    ADEBUG << "Get prediction done.";
  }

  auto status = frame_->Init(config_, time_stamp);
  if (!status.ok()) {
    AERROR << "failed to init frame";
    return Status(ErrorCode::PLANNING_ERROR, "init frame failed");
  }
  frame_->RecordInputDebug();
  return Status::OK();
}

Status Planning::Init() {
  pnc_map_.reset(new hdmap::PncMap(apollo::hdmap::BaseMapFile()));
  Frame::SetMap(pnc_map_.get());

  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                               &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_adapter_config_path);
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
  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() == nullptr) {
    std::string error_msg("Prediction is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }

  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->Init(
        pnc_map_.get(), config_.reference_line_smoother_config());
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

Status Planning::Start() {
  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->Start();
  }
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_planning_loop_rate), &Planning::OnTimer, this);
  return Status::OK();
}

void Planning::OnTimer(const ros::TimerEvent&) {
  RunOnce();
  if (frame_) {
    auto seq_num = frame_->SequenceNum();
    FrameHistory::instance()->Add(seq_num, std::move(frame_));
  }
}

void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb) {
  AdapterManager::FillPlanningHeader(Name(), trajectory_pb);
  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
  AdapterManager::PublishPlanning(*trajectory_pb);
}

void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb,
                                 double timestamp) {
  AdapterManager::FillPlanningHeader(Name(), trajectory_pb);
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
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
    PublishPlanningPb(&not_ready_pb);
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

  if (!status.ok()) {
    AERROR << "Update VehicleState failed.";
    not_ready->set_reason("Update VehicleState failed.");
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb);
    return;
  }

  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->UpdateRoutingResponse(
        AdapterManager::GetRoutingResponse()->GetLatestObserved());
  }

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  bool is_auto_mode = chassis.driving_mode() == chassis.COMPLETE_AUTO_DRIVE;
  const auto& stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          is_auto_mode, start_timestamp, planning_cycle_time,
          last_publishable_trajectory_);

  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, start_timestamp, stitching_trajectory.back());
  double end_timestamp = Clock::NowInSecond();
  double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  auto trajectory_pb = frame_->MutableADCTrajectory();
  trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(time_diff_ms);
  if (!status.ok()) {
    ADCTrajectory estop;
    estop.mutable_estop();
    AERROR << "Init frame failed";
    status.Save(estop.mutable_header()->mutable_status());
    PublishPlanningPb(&estop, start_timestamp);
    return;
  }

  status = Plan(start_timestamp, stitching_trajectory);

  end_timestamp = Clock::NowInSecond();
  time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << trajectory_pb->latency_stats().DebugString();

  if (status.ok()) {
    PublishPlanningPb(trajectory_pb, start_timestamp);
    ADEBUG << "Planning succeeded:" << trajectory_pb->header().DebugString();
  } else {
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    PublishPlanningPb(trajectory_pb, start_timestamp);
    AERROR << "Planning failed";
  }
}

void Planning::Stop() {
  last_publishable_trajectory_.Clear();
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  if (FLAGS_enable_reference_line_provider_thread) {
    ReferenceLineProvider::instance()->Stop();
  }
}

common::Status Planning::Plan(
    const double current_time_stamp,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory) {
  auto trajectory_pb = frame_->MutableADCTrajectory();
  if (FLAGS_enable_record_debug) {
    trajectory_pb->mutable_debug()
        ->mutable_planning_data()
        ->mutable_init_point()
        ->CopyFrom(stitching_trajectory.back());
  }
  auto status = Status::OK();
  for (auto& reference_line_info : frame_->reference_line_info()) {
    status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                            &reference_line_info);
    AERROR_IF(!status.ok()) << "planner failed to make a driving plan.";
  }

  const auto* best_reference_line = frame_->FindDriveReferenceLineInfo();
  if (!best_reference_line) {
    std::string msg("planner failed to make a driving plan");
    AERROR << msg;
    last_publishable_trajectory_.Clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  auto* ptr_debug = frame_->MutableADCTrajectory()->mutable_debug();
  ptr_debug->MergeFrom(best_reference_line->debug());
  frame_->MutableADCTrajectory()->mutable_latency_stats()->MergeFrom(
      best_reference_line->latency_stats());

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

  PublishableTrajectory publishable_trajectory(
      current_time_stamp, best_reference_line->trajectory());

  publishable_trajectory.PrependTrajectoryPoints(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1);

  publishable_trajectory.PopulateTrajectoryProtobuf(trajectory_pb);
  trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);

  // update last publishable trajectory;
  last_publishable_trajectory_ = std::move(publishable_trajectory);

  return status;
}

}  // namespace planning
}  // namespace apollo
