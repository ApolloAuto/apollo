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

#include <algorithm>
#include <list>
#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
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

OpenSpacePlanning::~OpenSpacePlanning() {
  planner_->Stop();
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  FrameHistory::Instance()->Clear();
  last_routing_.Clear();
}

std::string OpenSpacePlanning::Name() const { return "open_space_planning"; }

Status OpenSpacePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);

  planner_dispatcher_->Init();

  /*
    CHECK(apollo::common::util::GetProtoFromFile(
        FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
        << "Failed to load traffic rule config file "
        << FLAGS_traffic_rule_config_filename;
  */

  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map";

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

  std::list<hdmap::RouteSegments> segments;

  auto status = frame_->InitForOpenSpace();

  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }

  return Status::OK();
}

void OpenSpacePlanning::RunOnce(const LocalView& local_view,
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

  if (!IsVehicleStateValid(vehicle_state)) {
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    FillPlanningPb(start_timestamp, trajectory_pb);
    return;
  }

  if (IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    // TODO(QiL): Get latest parking info from new routing
  }

  // Update reference line provider and reset pull over if necessary
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  // planning is triggered by prediction data, but we can still use an estimated
  // cycle time for stitching
  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  std::vector<TrajectoryPoint> stitching_trajectory;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get());

  const uint32_t frame_num = seq_num_++;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
                     vehicle_state, trajectory_pb);

  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(trajectory_pb->mutable_debug());
  }

  trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  if (!status.ok()) {
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
      FillPlanningPb(start_timestamp, &estop_trajectory);
      trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(trajectory_pb->mutable_header()->mutable_status());
      FillPlanningPb(start_timestamp, trajectory_pb);
    }

    FillPlanningPb(start_timestamp, trajectory_pb);
    frame_->mutable_trajectory()->CopyFrom(*trajectory_pb);
    const uint32_t n = frame_->SequenceNum();
    FrameHistory::Instance()->Add(n, std::move(frame_));
    return;
  }

  status = Plan(start_timestamp, stitching_trajectory, trajectory_pb);

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
  FillPlanningPb(start_timestamp, trajectory_pb);
  ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();

  frame_->mutable_trajectory()->CopyFrom(*trajectory_pb);
  const uint32_t n = frame_->SequenceNum();
  FrameHistory::Instance()->Add(n, std::move(frame_));
}

Status OpenSpacePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get());

  if (status.ok() && FLAGS_enable_record_debug &&
      FLAGS_open_space_planner_switchable) {
    frame_.get()->RecordOpenSpacePlannerDebug(ptr_debug);
  }

  // TODO(QiL): update last_publishable_trajectory_ for stitching
  trajectory_pb->CopyFrom(frame_->trajectory());

  return status;
}

bool OpenSpacePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  // TODO(All): check config params
  return true;
}

}  // namespace planning
}  // namespace apollo
