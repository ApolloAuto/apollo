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

#include "google/protobuf/repeated_field.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
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

const Frame* Planning::GetFrame() const { return frame_.get(); }
const hdmap::PncMap* Planning::GetPncMap() const { return pnc_map_.get(); }

bool Planning::InitFrame(const uint32_t sequence_num) {
  frame_.reset(new Frame(sequence_num));
  if (AdapterManager::GetRoutingResult()->Empty()) {
    AERROR << "Routing is empty";
    return false;
  }
  common::TrajectoryPoint point;
  frame_->SetVehicleInitPose(VehicleState::instance()->pose());
  frame_->SetRoutingResult(
      AdapterManager::GetRoutingResult()->GetLatestObserved());
  if (FLAGS_enable_prediction && !AdapterManager::GetPrediction()->Empty()) {
    const auto& prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();
    frame_->SetPrediction(prediction);
    ADEBUG << "Get prediction";
  }

  if (!frame_->Init()) {
    AERROR << "failed to init frame";
    return false;
  }
  return true;
}

Status Planning::Init() {
  pnc_map_.reset(new hdmap::PncMap(FLAGS_map_filename));
  Frame::SetMap(pnc_map_.get());

  if (!apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                              &config_)) {
    AERROR << "failed to load planning config file "
           << FLAGS_planning_config_file;
    return Status(
        ErrorCode::PLANNING_ERROR,
        "failed to load planning config file: " + FLAGS_planning_config_file);
  }

  AdapterManager::Init(FLAGS_adapter_config_path);
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
  if (AdapterManager::GetRoutingResult() == nullptr) {
    std::string error_msg("RoutingResult is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  // TODO(all) temporarily use the offline routing data.
  if (!AdapterManager::GetRoutingResult()->HasReceived()) {
    if (!AdapterManager::GetRoutingResult()->FeedFile(
            FLAGS_offline_routing_file)) {
      auto error_msg = common::util::StrCat(
          "Failed to load offline routing file ", FLAGS_offline_routing_file);
      AERROR << error_msg;
      return Status(ErrorCode::PLANNING_ERROR, error_msg);
    } else {
      AWARN << "Using offline routing file " << FLAGS_offline_routing_file;
    }
  }
  if (AdapterManager::GetPrediction() == nullptr) {
    std::string error_msg("Prediction is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
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
  static ros::Rate loop_rate(FLAGS_planning_loop_rate);
  while (ros::ok()) {
    RunOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return Status::OK();
}

void Planning::RecordInput(ADCTrajectory* trajectory_pb) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto planning_data = trajectory_pb->mutable_debug()->mutable_planning_data();
  auto adc_position = planning_data->mutable_adc_position();
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  adc_position->CopyFrom(localization);

  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  auto debug_chassis = planning_data->mutable_chassis();
  debug_chassis->CopyFrom(chassis);

  const auto& routing_result =
      AdapterManager::GetRoutingResult()->GetLatestObserved();

  auto debug_routing = planning_data->mutable_routing();
  debug_routing->CopyFrom(routing_result);
}

void Planning::RunOnce() {
  AdapterManager::Observe();
  if (AdapterManager::GetLocalization()->Empty()) {
    AERROR << "Localization is not available; skip the planning cycle";
    return;
  }

  if (AdapterManager::GetChassis()->Empty()) {
    AERROR << "Chassis is not available; skip the planning cycle";
    return;
  }
  if (AdapterManager::GetRoutingResult()->Empty()) {
    AERROR << "RoutingResult is not available; skip the planning cycle";
    return;
  }

  // FIXME(all): enable prediction check when perception and prediction is
  // ready.
  // if (FLAGS_enable_prediction && AdapterManager::GetPrediction()->Empty()) {
  //   AERROR << "Prediction is not available; skip the planning cycle";
  //   return;
  // }

  AINFO << "Start planning ...";
  const double start_timestamp = apollo::common::time::ToSecond(Clock::Now());

  // localization
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();

  // chassis
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();

  common::VehicleState::instance()->Update(localization, chassis);

  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  // the execution_start_time is the estimated time when the planned trajectory
  // will be executed by the controller.
  double execution_start_time =
      apollo::common::time::ToSecond(apollo::common::time::Clock::Now()) +
      planning_cycle_time;

  ADCTrajectory trajectory_pb;
  RecordInput(&trajectory_pb);

  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  if (!InitFrame(frame_num)) {
    AERROR << "Init frame failed";
    return;
  }

  bool is_auto_mode = chassis.driving_mode() == chassis.COMPLETE_AUTO_DRIVE;
  bool res_planning = Plan(is_auto_mode, execution_start_time, &trajectory_pb);

  const double end_timestamp = apollo::common::time::ToSecond(Clock::Now());
  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  trajectory_pb.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: " << trajectory_pb.latency_stats().DebugString();

  if (res_planning) {
    AdapterManager::FillPlanningHeader("planning", &trajectory_pb);
    trajectory_pb.mutable_header()->set_timestamp_sec(execution_start_time);
    AdapterManager::PublishPlanning(trajectory_pb);
    ADEBUG << "Planning succeeded:" << trajectory_pb.header().DebugString();
  } else {
    AERROR << "Planning failed";
  }
}

void Planning::Stop() {}

bool Planning::Plan(const bool is_on_auto_mode, const double publish_time,
                    ADCTrajectory* trajectory_pb) {
  double vehicle_state_time = VehicleState::instance()->timestamp();
  const auto& stitching_trajectory =
      TrajectoryStitcher::compute_stitching_trajectory(
          is_on_auto_mode, vehicle_state_time, last_publishable_trajectory_);

  frame_->SetPlanningStartPoint(stitching_trajectory.back());

  PublishableTrajectory publishable_trajectory;
  auto status = planner_->Plan(frame_.get(), &publishable_trajectory);

  if (FLAGS_enable_record_debug) {
    trajectory_pb->mutable_debug()
        ->mutable_planning_data()
        ->mutable_init_point()
        ->CopyFrom(stitching_trajectory.back());
    trajectory_pb->mutable_debug()->mutable_planning_data()->set_is_replan(
        stitching_trajectory.size() == 1);

    // TODO: check whether current planner is EM planner.
    auto& em_debugger =
        dynamic_cast<EMPlanner*>(planner_.get())->em_planner_debugger();

    for (auto it = em_debugger.paths_.begin(); it != em_debugger.paths_.end();
         ++it) {
      auto ptr_path_planning_info =
          trajectory_pb->mutable_debug()->mutable_planning_data()->add_path();
      ptr_path_planning_info->set_name(it->first);
      ptr_path_planning_info->mutable_path_point()->CopyFrom(
          {it->second.first.points().begin(), it->second.first.points().end()});

      auto ptr_stats =
          trajectory_pb->mutable_latency_stats()->add_processor_stats();
      ptr_stats->set_name(it->first);
      ptr_stats->set_time_ms(it->second.second);
    }

    for (auto it = em_debugger.speed_profiles_.begin();
         it != em_debugger.speed_profiles_.end(); ++it) {
      auto ptr_speed_planning_info = trajectory_pb->mutable_debug()
                                         ->mutable_planning_data()
                                         ->add_speed_plan();
      ptr_speed_planning_info->set_name(it->first);
      ptr_speed_planning_info->mutable_speed_point()->CopyFrom(
          {it->second.first.begin(), it->second.first.end()});

      auto ptr_stats =
          trajectory_pb->mutable_latency_stats()->add_processor_stats();
      ptr_stats->set_name(it->first);
      ptr_stats->set_time_ms(it->second.second);
    }

    auto ptr_reference_line =
        trajectory_pb->mutable_debug()->mutable_planning_data()->add_path();
    ptr_reference_line->set_name("planning_reference_line");

    ptr_reference_line->mutable_path_point()->CopyFrom(
        {em_debugger.reference_line_.begin(),
         em_debugger.reference_line_.end()});
  }

  if (status != Status::OK()) {
    AERROR << "planner failed to make a driving plan";
    last_publishable_trajectory_.Clear();
    return false;
  }

  publishable_trajectory.prepend_trajectory_points(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1);

  publishable_trajectory.set_header_time(vehicle_state_time);

  publishable_trajectory.populate_trajectory_protobuf(trajectory_pb);

  // update last publishable trajectory;
  last_publishable_trajectory_ = std::move(publishable_trajectory);
  return true;
}

}  // namespace planning
}  // namespace apollo
