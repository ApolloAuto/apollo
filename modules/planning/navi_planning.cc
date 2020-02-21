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

#include "modules/planning/navi_planning.h"

#include <algorithm>
#include <list>
#include <map>

#include "cyber/common/file.h"
#include "google/protobuf/repeated_field.h"

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/planner/navi/navi_planner.h"
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
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;

NaviPlanning::~NaviPlanning() {
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  FrameHistory::Instance()->Clear();
  History::Instance()->Clear();
  PlanningContext::Instance()->mutable_planning_status()->Clear();
}

std::string NaviPlanning::Name() const { return "navi_planning"; }

Status NaviPlanning::Init(const PlanningConfig& config) {
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

  // clear planning history
  History::Instance()->Clear();

  // clear planning status
  PlanningContext::Instance()->mutable_planning_status()->Clear();

  planner_ = planner_dispatcher_->DispatchPlanner();
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  return planner_->Init(config_);
}

Status NaviPlanning::InitFrame(const uint32_t sequence_num,
                               const TrajectoryPoint& planning_start_point,
                               const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
    std::string msg = "Failed to create reference line";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  auto status = frame_->Init(reference_lines, segments,
                             reference_line_provider_->FutureRouteWaypoints());

  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

void NaviPlanning::RunOnce(const LocalView& local_view,
                           ADCTrajectory* const trajectory_pb) {
  local_view_ = local_view;
  const double start_timestamp = Clock::NowInSeconds();

  // recreate reference line provider in every cycle
  hdmap_ = HDMapUtil::BaseMapPtr(*local_view.relative_map);
  // Prefer "std::make_unique" to direct use of "new".
  // Refer to "https://herbsutter.com/gotw/_102/" for details.
  reference_line_provider_ =
      std::make_unique<ReferenceLineProvider>(hdmap_, local_view_.relative_map);

  // localization
  ADEBUG << "Get localization: "
         << local_view_.localization_estimate->DebugString();

  // chassis
  ADEBUG << "Get chassis: " << local_view_.chassis->DebugString();

  Status status = VehicleStateProvider::Instance()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);

  auto vehicle_config =
      ComputeVehicleConfigFromLocalization(*local_view_.localization_estimate);

  if (last_vehicle_config_.is_valid_ && vehicle_config.is_valid_) {
    auto x_diff_map = vehicle_config.x_ - last_vehicle_config_.x_;
    auto y_diff_map = vehicle_config.y_ - last_vehicle_config_.y_;

    auto cos_map_veh = std::cos(last_vehicle_config_.theta_);
    auto sin_map_veh = std::sin(last_vehicle_config_.theta_);

    auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
    auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

    auto theta_diff = vehicle_config.theta_ - last_vehicle_config_.theta_;

    TrajectoryStitcher::TransformLastPublishedTrajectory(
        x_diff_veh, y_diff_veh, theta_diff, last_publishable_trajectory_.get());
  }
  last_vehicle_config_ = vehicle_config;

  VehicleState vehicle_state =
      VehicleStateProvider::Instance()->vehicle_state();

  // estimate (x, y) at current timestamp
  // This estimate is only valid if the current time and vehicle state timestamp
  // differs only a small amount (20ms). When the different is too large, the
  // estimation is invalid.
  DCHECK_GE(start_timestamp, vehicle_state.timestamp());
  if (start_timestamp - vehicle_state.timestamp() <
      FLAGS_message_latency_threshold) {
    auto future_xy = VehicleStateProvider::Instance()->EstimateFuturePosition(
        start_timestamp - vehicle_state.timestamp());
    vehicle_state.set_x(future_xy.x());
    vehicle_state.set_y(future_xy.y());
    vehicle_state.set_timestamp(start_timestamp);
  }

  auto* not_ready = trajectory_pb->mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    // TODO(all): integrate reverse gear
    trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, trajectory_pb);
    return;
  }

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  std::vector<TrajectoryPoint> stitching_trajectory;
  std::string replan_reason;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      FLAGS_trajectory_stitching_preserved_length, true,
      last_publishable_trajectory_.get(), &replan_reason);

  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  if (!frame_) {
    std::string msg("Failed to init frame");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(trajectory_pb->mutable_header()->mutable_status());
    // TODO(all): integrate reverse gear
    trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, trajectory_pb);
    return;
  }

  EgoInfo::Instance()->Update(stitching_trajectory.back(), vehicle_state);

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
      // TODO(all): integrate reverse gear
      trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
      FillPlanningPb(start_timestamp, &estop_trajectory);
    } else {
      trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(trajectory_pb->mutable_header()->mutable_status());
      // TODO(all): integrate reverse gear
      trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
      FillPlanningPb(start_timestamp, trajectory_pb);
    }

    frame_->set_current_frame_planned_trajectory(*trajectory_pb);
    auto seq_num = frame_->SequenceNum();
    FrameHistory::Instance()->Add(seq_num, std::move(frame_));

    return;
  }

  // Use planning pad message to make driving decisions
  if (FLAGS_enable_planning_pad_msg) {
    const auto& pad_msg_driving_action = frame_->GetPadMsgDrivingAction();
    ProcessPadMsg(pad_msg_driving_action);
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
  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
  FillPlanningPb(start_timestamp, trajectory_pb);
  ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();

  auto seq_num = frame_->SequenceNum();
  FrameHistory::Instance()->Add(seq_num, std::move(frame_));
}

void NaviPlanning::ProcessPadMsg(DrivingAction drvie_action) {
  if (config_.has_navigation_planning_config()) {
    std::map<std::string, uint32_t> lane_id_to_priority;
    auto& ref_line_info_group = *frame_->mutable_reference_line_info();
    if (drvie_action != DrivingAction::NONE) {
      using LaneInfoPair = std::pair<std::string, double>;
      std::string current_lane_id;
      switch (drvie_action) {
        case DrivingAction::FOLLOW: {
          AINFO << "Received follow drive action";
          std::string current_lane_id = GetCurrentLaneId();
          if (!current_lane_id.empty()) {
            target_lane_id_ = current_lane_id;
          }
          break;
        }
        case DrivingAction::CHANGE_LEFT: {
          AINFO << "Received change left lane drive action";
          std::vector<LaneInfoPair> lane_info_group;
          GetLeftNeighborLanesInfo(&lane_info_group);
          if (!lane_info_group.empty()) {
            target_lane_id_ = lane_info_group.front().first;
          }
          break;
        }
        case DrivingAction::CHANGE_RIGHT: {
          AINFO << "Received change right lane drive action";
          std::vector<LaneInfoPair> lane_info_group;
          GetRightNeighborLanesInfo(&lane_info_group);
          if (!lane_info_group.empty()) {
            target_lane_id_ = lane_info_group.front().first;
          }
          break;
        }
        case DrivingAction::PULL_OVER: {
          AINFO << "Received pull over drive action";
          // to do
          break;
        }
        case DrivingAction::STOP: {
          AINFO << "Received stop drive action";
          // to do
          break;
        }
        default: {
          AWARN << "Received undefined drive action.";
          break;
        }
      }
    }

    if (!target_lane_id_.empty()) {
      static constexpr uint32_t KTargetRefLinePriority = 0;
      static constexpr uint32_t kOtherRefLinePriority = 10;
      for (auto& ref_line_info : ref_line_info_group) {
        auto lane_id = ref_line_info.Lanes().Id();
        ADEBUG << "lane_id : " << lane_id;
        lane_id_to_priority[lane_id] = kOtherRefLinePriority;
        if (lane_id == target_lane_id_) {
          lane_id_to_priority[lane_id] = KTargetRefLinePriority;
          ADEBUG << "target lane_id : " << lane_id;
        }
      }
      frame_->UpdateReferenceLinePriority(lane_id_to_priority);
    }
  }

  // other planner to do
}

std::string NaviPlanning::GetCurrentLaneId() {
  auto& ref_line_info_group = *frame_->mutable_reference_line_info();
  const auto& vehicle_state = frame_->vehicle_state();
  common::math::Vec2d adc_position(vehicle_state.x(), vehicle_state.y());
  std::string current_lane_id;
  for (auto& ref_line_info : ref_line_info_group) {
    auto lane_id = ref_line_info.Lanes().Id();
    auto& ref_line = ref_line_info.reference_line();
    if (ref_line.IsOnLane(adc_position)) {
      current_lane_id = lane_id;
    }
  }
  return current_lane_id;
}

void NaviPlanning::GetLeftNeighborLanesInfo(
    std::vector<std::pair<std::string, double>>* const lane_info_group) {
  auto& ref_line_info_group = *frame_->mutable_reference_line_info();
  const auto& vehicle_state = frame_->vehicle_state();
  for (auto& ref_line_info : ref_line_info_group) {
    common::math::Vec2d adc_position(vehicle_state.x(), vehicle_state.y());
    auto& ref_line = ref_line_info.reference_line();
    if (ref_line.IsOnLane(adc_position)) {
      continue;
    }
    auto lane_id = ref_line_info.Lanes().Id();
    auto ref_point =
        ref_line.GetReferencePoint(vehicle_state.x(), vehicle_state.y());
    double y = ref_point.y();
    // in FLU positive on the left
    if (y > 0.0) {
      lane_info_group->emplace_back(lane_id, y);
    }
  }
  // sort neighbor lanes from near to far
  using LaneInfoPair = std::pair<std::string, double>;
  std::sort(lane_info_group->begin(), lane_info_group->end(),
            [](const LaneInfoPair& left, const LaneInfoPair& right) {
              return left.second < right.second;
            });
}

void NaviPlanning::GetRightNeighborLanesInfo(
    std::vector<std::pair<std::string, double>>* const lane_info_group) {
  auto& ref_line_info_group = *frame_->mutable_reference_line_info();
  const auto& vehicle_state = frame_->vehicle_state();
  for (auto& ref_line_info : ref_line_info_group) {
    common::math::Vec2d adc_position(vehicle_state.x(), vehicle_state.y());
    auto& ref_line = ref_line_info.reference_line();
    if (ref_line.IsOnLane(adc_position)) {
      continue;
    }
    auto lane_id = ref_line_info.Lanes().Id();
    auto ref_point =
        ref_line.GetReferencePoint(vehicle_state.x(), vehicle_state.y());
    double y = ref_point.y();
    // in FLU negative on the right
    if (y < 0.0) {
      lane_info_group->emplace_back(lane_id, y);
    }
  }

  // sort neighbor lanes from near to far
  using LaneInfoPair = std::pair<std::string, double>;
  std::sort(lane_info_group->begin(), lane_info_group->end(),
            [](const LaneInfoPair& left, const LaneInfoPair& right) {
              return left.second > right.second;
            });
}

void NaviPlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
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

Status NaviPlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
  }

  auto status =
      planner_->Plan(stitching_trajectory.back(), frame_.get(), trajectory_pb);

  ExportReferenceLineDebug(ptr_debug);

  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (!best_ref_info) {
    std::string msg("planner failed to make a driving plan");
    AERROR << msg;
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ptr_debug->MergeFrom(best_ref_info->debug());
  trajectory_pb->mutable_latency_stats()->MergeFrom(
      best_ref_info->latency_stats());
  // set right of way status
  trajectory_pb->set_right_of_way_status(best_ref_info->GetRightOfWayStatus());
  for (const auto& id : best_ref_info->TargetLaneId()) {
    trajectory_pb->add_lane_id()->CopyFrom(id);
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

  ADEBUG << "current_time_stamp: " << current_time_stamp;

  // Navi Planner doesn't need to stitch the last path planning
  // trajectory.Otherwise, it will cause the Dreamview planning track to display
  // flashing or bouncing
  // TODO(Yifei): remove this if navi-planner doesn't need stitching
  /**
  if (FLAGS_enable_stitch_last_trajectory) {
    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));
  }
  **/

  for (size_t i = 0; i < last_publishable_trajectory_->NumOfPoints(); ++i) {
    if (last_publishable_trajectory_->TrajectoryPointAt(i).relative_time() >
        FLAGS_trajectory_time_high_density_period) {
      break;
    }
    ADEBUG << last_publishable_trajectory_->TrajectoryPointAt(i)
                  .ShortDebugString();
  }

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  best_ref_info->ExportEngageAdvice(trajectory_pb->mutable_engage_advice());

  return status;
}

/*void NaviPlanning::Stop() {
  AWARN << "Planning Stop is called";
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  FrameHistory::Instance()->Clear();
 PlanningContext::Instance()->mutable_planning_status()->Clear();
}*/

NaviPlanning::VehicleConfig NaviPlanning::ComputeVehicleConfigFromLocalization(
    const localization::LocalizationEstimate& localization) const {
  NaviPlanning::VehicleConfig vehicle_config;

  if (!localization.pose().has_position()) {
    return vehicle_config;
  }

  vehicle_config.x_ = localization.pose().position().x();
  vehicle_config.y_ = localization.pose().position().y();

  const auto& orientation = localization.pose().orientation();

  if (localization.pose().has_heading()) {
    vehicle_config.theta_ = localization.pose().heading();
  } else {
    vehicle_config.theta_ = common::math::QuaternionToHeading(
        orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
  }

  vehicle_config.is_valid_ = true;
  return vehicle_config;
}

bool NaviPlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_navigation_planning_config()) {
    return false;
  }
  // TODO(All): check other config params

  return true;
}

}  // namespace planning
}  // namespace apollo
