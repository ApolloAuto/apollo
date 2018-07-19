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
#include <list>
#include <vector>

#include "google/protobuf/repeated_field.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/traffic_decider/traffic_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;

Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

#define CHECK_ADAPTER(NAME)                                               \
  if (AdapterManager::Get##NAME() == nullptr) {                           \
    AERROR << #NAME << " is not registered";                              \
    return Status(ErrorCode::PLANNING_ERROR, #NAME " is not registered"); \
  }

#define CHECK_ADAPTER_IF(CONDITION, NAME) \
  if (CONDITION) CHECK_ADAPTER(NAME)

void Planning::RegisterPlanners() {
  planner_factory_.Register(
      PlanningConfig::RTK, []() -> Planner* { return new RTKReplayPlanner(); });
  planner_factory_.Register(PlanningConfig::EM,
                            []() -> Planner* { return new EMPlanner(); });
  planner_factory_.Register(PlanningConfig::LATTICE,
                            []() -> Planner* { return new LatticePlanner(); });
  planner_factory_.Register(PlanningConfig::NAVI,
                            []() -> Planner* { return new NaviPlanner(); });
}

Status Planning::InitFrame(const uint32_t sequence_num,
                           const TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,
                         vehicle_state, reference_line_provider_.get()));
  auto status = frame_->Init();
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

void Planning::ResetPullOver(const routing::RoutingResponse& response) {
  auto* pull_over =
      util::GetPlanningStatus()->mutable_planning_state()->mutable_pull_over();
  if (!last_routing_.has_header()) {
    last_routing_ = response;
    pull_over->Clear();
    return;
  }
  if (!pull_over->in_pull_over()) {
    return;
  }
  if (hdmap::PncMap::IsNewRouting(last_routing_, response)) {
    pull_over->Clear();
    last_routing_ = response;
    AINFO << "Cleared Pull Over Status after received new routing";
  }
}

void Planning::CheckPlanningConfig() {
  if (config_.has_em_planner_config() &&
      config_.em_planner_config().has_dp_st_speed_config()) {
    const auto& dp_st_speed_config =
        config_.em_planner_config().dp_st_speed_config();
    CHECK(dp_st_speed_config.has_matrix_dimension_s());
    CHECK_GT(dp_st_speed_config.matrix_dimension_s(), 3);
    CHECK_LT(dp_st_speed_config.matrix_dimension_s(), 10000);
    CHECK(dp_st_speed_config.has_matrix_dimension_t());
    CHECK_GT(dp_st_speed_config.matrix_dimension_t(), 3);
    CHECK_LT(dp_st_speed_config.matrix_dimension_t(), 10000);
  }
  // TODO(All): check other config params
}

Status Planning::Init() {
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                               &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  CheckPlanningConfig();

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;

  // initialize planning thread pool
  PlanningThreadPool::instance()->Init();

  // clear planning status
  util::GetPlanningStatus()->Clear();

  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_planning_adapter_config_filename);
  }
  CHECK_ADAPTER(Localization);
  CHECK_ADAPTER(Chassis);
  CHECK_ADAPTER(RoutingResponse);
  CHECK_ADAPTER(RoutingRequest);
  CHECK_ADAPTER_IF(FLAGS_use_navigation_mode, RelativeMap);
  CHECK_ADAPTER_IF(FLAGS_use_navigation_mode && FLAGS_enable_prediction,
                   PerceptionObstacles);
  CHECK_ADAPTER_IF(FLAGS_enable_prediction, Prediction);
  CHECK_ADAPTER(TrafficLightDetection);

  if (!FLAGS_use_navigation_mode) {
    hdmap_ = HDMapUtil::BaseMapPtr();
    CHECK(hdmap_) << "Failed to load map";
    // Prefer "std::make_unique" to direct use of "new".
    // Reference "https://herbsutter.com/gotw/_102/" for details.
    reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
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
  // The "reference_line_provider_" may not be created yet in navigation mode.
  // It is necessary to check its existence.
  if (reference_line_provider_) {
    reference_line_provider_->Start();
  }
  start_time_ = Clock::NowInSeconds();
  AINFO << "Planning started";
  return Status::OK();
}

void Planning::OnTimer(const ros::TimerEvent&) {
  RunOnce();

  if (FLAGS_planning_test_mode && FLAGS_test_duration > 0.0 &&
      Clock::NowInSeconds() - start_time_ > FLAGS_test_duration) {
    ros::shutdown();
  }
}

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

  if (FLAGS_use_planning_fallback &&
      trajectory_pb->trajectory_point_size() == 0) {
    SetFallbackTrajectory(trajectory_pb);
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
  // snapshot all coming data
  AdapterManager::Observe();

  const double start_timestamp = Clock::NowInSeconds();

  ADCTrajectory not_ready_pb;
  auto* not_ready = not_ready_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();
  if (AdapterManager::GetLocalization()->Empty()) {
    not_ready->set_reason("localization not ready");
  } else if (AdapterManager::GetChassis()->Empty()) {
    not_ready->set_reason("chassis not ready");
  } else if (!FLAGS_use_navigation_mode &&
             AdapterManager::GetRoutingResponse()->Empty()) {
    not_ready->set_reason("routing not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  }
  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  if (FLAGS_use_navigation_mode) {
    // recreate reference line provider in every cycle
    hdmap_ = HDMapUtil::BaseMapPtr();
    // Prefer "std::make_unique" to direct use of "new".
    // Reference "https://herbsutter.com/gotw/_102/" for details.
    reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
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

  if (FLAGS_use_navigation_mode) {
    auto vehicle_config = ComputeVehicleConfigFromLocalization(localization);

    if (last_vehicle_config_.is_valid_ && vehicle_config.is_valid_) {
      auto x_diff_map = vehicle_config.x_ - last_vehicle_config_.x_;
      auto y_diff_map = vehicle_config.y_ - last_vehicle_config_.y_;

      auto cos_map_veh = std::cos(last_vehicle_config_.theta_);
      auto sin_map_veh = std::sin(last_vehicle_config_.theta_);

      auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
      auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

      auto theta_diff = vehicle_config.theta_ - last_vehicle_config_.theta_;

      TrajectoryStitcher::TransformLastPublishedTrajectory(
          x_diff_veh, y_diff_veh, theta_diff,
          last_publishable_trajectory_.get());
    }
    last_vehicle_config_ = vehicle_config;
  }

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
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  if (!FLAGS_use_navigation_mode &&
      !reference_line_provider_->UpdateRoutingResponse(
          AdapterManager::GetRoutingResponse()->GetLatestObserved())) {
    std::string msg("Failed to update routing in reference line provider");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }

  if (FLAGS_enable_prediction && AdapterManager::GetPrediction()->Empty()) {
    AWARN_EVERY(100) << "prediction is enabled but no prediction provided";
  }

  // Update reference line provider and reset pull over if necessary
  if (!FLAGS_use_navigation_mode) {
    reference_line_provider_->UpdateVehicleState(vehicle_state);
    ResetPullOver(AdapterManager::GetRoutingResponse()->GetLatestObserved());
  }

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  bool is_replan = false;
  std::vector<TrajectoryPoint> stitching_trajectory;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get(), &is_replan);

  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
                     vehicle_state);
  if (!frame_) {
    std::string msg("Failed to init frame");
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);
    return;
  }
  auto* trajectory_pb = frame_->mutable_trajectory();
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
      PublishPlanningPb(&estop_trajectory, start_timestamp);
    } else {
      trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(trajectory_pb->mutable_header()->mutable_status());
      PublishPlanningPb(trajectory_pb, start_timestamp);
    }

    auto seq_num = frame_->SequenceNum();
    FrameHistory::instance()->Add(seq_num, std::move(frame_));

    return;
  }

  for (auto& ref_line_info : frame_->reference_line_info()) {
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

  trajectory_pb->set_is_replan(is_replan);
  PublishPlanningPb(trajectory_pb, start_timestamp);
  ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();

  auto seq_num = frame_->SequenceNum();
  FrameHistory::instance()->Add(seq_num, std::move(frame_));
}

void Planning::SetFallbackTrajectory(ADCTrajectory* trajectory_pb) {
  CHECK_NOTNULL(trajectory_pb);

  if (FLAGS_use_navigation_mode) {
    const double v = VehicleStateProvider::instance()->linear_velocity();
    for (double t = 0.0; t < FLAGS_navigation_fallback_cruise_time; t += 0.1) {
      const double s = t * v;

      auto* cruise_point = trajectory_pb->add_trajectory_point();
      cruise_point->mutable_path_point()->CopyFrom(
          common::util::MakePathPoint(s, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      cruise_point->mutable_path_point()->set_s(s);
      cruise_point->set_v(v);
      cruise_point->set_a(0.0);
      cruise_point->set_relative_time(t);
    }
  } else {
    // use planning trajecotry from last cycle
    auto* last_planning = AdapterManager::GetPlanning();
    if (last_planning != nullptr && !last_planning->Empty()) {
      const auto& traj = last_planning->GetLatestObserved();

      const double current_time_stamp = trajectory_pb->header().timestamp_sec();
      const double pre_time_stamp = traj.header().timestamp_sec();

      for (int i = 0; i < traj.trajectory_point_size(); ++i) {
        const double t = traj.trajectory_point(i).relative_time() +
                         pre_time_stamp - current_time_stamp;
        auto* p = trajectory_pb->add_trajectory_point();
        p->CopyFrom(traj.trajectory_point(i));
        p->set_relative_time(t);
      }
    }
  }
}

void Planning::Stop() {
  AERROR << "Planning Stop is called";
  // PlanningThreadPool::instance()->Stop();
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  FrameHistory::instance()->Clear();
}

void Planning::SetLastPublishableTrajectory(
    const ADCTrajectory& adc_trajectory) {
  last_publishable_trajectory_.reset(new PublishableTrajectory(adc_trajectory));
}

void Planning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  for (auto& reference_line_info : frame_->reference_line_info()) {
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

Status Planning::Plan(const double current_time_stamp,
                      const std::vector<TrajectoryPoint>& stitching_trajectory,
                      ADCTrajectory* trajectory_pb) {
  auto* ptr_debug = trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get());

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

  best_ref_info->ExportEngageAdvice(trajectory_pb->mutable_engage_advice());

  return status;
}

Planning::VehicleConfig Planning::ComputeVehicleConfigFromLocalization(
    const localization::LocalizationEstimate& localization) const {
  Planning::VehicleConfig vehicle_config;

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

}  // namespace planning
}  // namespace apollo
