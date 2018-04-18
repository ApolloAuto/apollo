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

/**
 * @file
 **/

#include "modules/planning/common/reference_line_info.h"

#include <algorithm>
#include <functional>
#include <utility>

#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::SLPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::planning::util::GetPlanningStatus;

ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      lanes_(segments) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }
  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }
  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }
  is_on_reference_line_ = reference_line_.IsOnRoad(adc_sl_boundary_);
  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  if (hdmap::GetSpeedControls()) {
    auto* speed_controls = hdmap::GetSpeedControls();
    for (const auto& speed_control : speed_controls->speed_control()) {
      reference_line_.AddSpeedLimit(speed_control);
    }
  }

  // set lattice planning target speed limit;
  double cruise_speed =
      std::min(FLAGS_speed_upper_bound,
               reference_line().GetSpeedLimitFromS(adc_sl_boundary_.end_s()));
  SetCruiseSpeed(std::min(FLAGS_default_cruise_speed, cruise_speed));
  is_inited_ = true;
  return true;
}

bool ReferenceLineInfo::IsInited() const { return is_inited_; }

bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
  constexpr double kEpsilon = 1e-2;
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
}

void ReferenceLineInfo::SetJunctionRightOfWay(double junction_s,
                                              bool is_protected) {
  auto* right_of_way = GetPlanningStatus()->mutable_right_of_way();
  auto* junction_right_of_way = right_of_way->mutable_junction();
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s)) {
      (*junction_right_of_way)[overlap.object_id] = is_protected;
    }
  }
}

ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  auto* right_of_way = GetPlanningStatus()->mutable_right_of_way();
  auto* junction_right_of_way = right_of_way->mutable_junction();
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (overlap.end_s < adc_sl_boundary_.start_s()) {
      junction_right_of_way->erase(overlap.object_id);
    } else if (WithinOverlap(overlap, adc_sl_boundary_.end_s())) {
      auto is_protected = (*junction_right_of_way)[overlap.object_id];
      if (is_protected) {
        return ADCTrajectory::PROTECTED;
      } else {
        double junction_s = (overlap.end_s + overlap.start_s) / 2.0;
        auto ref_point = reference_line_.GetReferencePoint(junction_s);
        if (ref_point.lane_waypoints().empty()) {
          return ADCTrajectory::PROTECTED;
        }
        for (const auto& waypoint : ref_point.lane_waypoints()) {
          if (waypoint.lane->lane().turn() == hdmap::Lane::NO_TURN) {
            return ADCTrajectory::PROTECTED;
          }
        }
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;
}

const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }

const std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {
  std::list<hdmap::Id> lane_ids;
  for (const auto& lane_seg : lanes_) {
    lane_ids.push_back(lane_seg.lane->id());
  }
  return lane_ids;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}
const TrajectoryPoint& ReferenceLineInfo::AdcPlanningPoint() const {
  return adc_planning_point_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

void ReferenceLineInfo::AddObstacleHelper(const Obstacle* obstacle, int* ret) {
  auto* path_obstacle = AddObstacle(obstacle);
  *ret = path_obstacle == nullptr ? 0 : 1;
}

// AddObstacle is thread safe
PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
  if (!path_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                     &perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);

  if (IsUnrelaventObstacle(path_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
  } else {
    ADEBUG << "build reference line st boundary. id:" << obstacle->Id();
    path_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                                adc_sl_boundary_.start_s());

    ADEBUG << "reference line st boundary: "
           << path_obstacle->reference_line_st_boundary().min_t() << ", "
           << path_obstacle->reference_line_st_boundary().max_t()
           << ", s_max: " << path_obstacle->reference_line_st_boundary().max_s()
           << ", s_min: "
           << path_obstacle->reference_line_st_boundary().min_s();
  }
  return path_obstacle;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  if (FLAGS_use_multi_thread_to_add_obstacles) {
    std::vector<int> ret(obstacles.size(), 0);
    for (size_t i = 0; i < obstacles.size(); ++i) {
      const auto* obstacle = obstacles.at(i);
      PlanningThreadPool::instance()->Push(std::bind(
          &ReferenceLineInfo::AddObstacleHelper, this, obstacle, &(ret[i])));
    }
    PlanningThreadPool::instance()->Synchronize();
    if (std::find(ret.begin(), ret.end(), 0) != ret.end()) {
      return false;
    }
  } else {
    for (const auto* obstacle : obstacles) {
      if (!AddObstacle(obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
        return false;
      }
    }
  }
  return true;
}

bool ReferenceLineInfo::IsUnrelaventObstacle(PathObstacle* path_obstacle) {
  // if adc is on the road, and obstacle behind adc, ignore
  if (path_obstacle->PerceptionSLBoundary().end_s() >
      reference_line_.Length()) {
    return true;
  }
  if (is_on_reference_line_ &&
      path_obstacle->PerceptionSLBoundary().end_s() <
          adc_sl_boundary_.end_s() &&
      reference_line_.IsOnRoad(path_obstacle->PerceptionSLBoundary())) {
    return true;
  }
  return false;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

double ReferenceLineInfo::TrajectoryLength() const {
  const auto& tps = discretized_trajectory_.trajectory_points();
  if (tps.empty()) {
    return 0.0;
  }
  return tps.back().path_point().s();
}

void ReferenceLineInfo::SetStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

void ReferenceLineInfo::SetCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnRoad(sl_point);
}

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  CHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
  if (path_data_.discretized_path().NumOfPoints() == 0) {
    AWARN << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point;
    if (!path_data_.GetPathPointWithPathS(speed_point.s(), &path_point)) {
      AERROR << "Fail to get path data with s " << speed_point.s()
             << "path total length " << path_data_.discretized_path().Length();
      return false;
    }
    path_point.set_s(path_point.s() + start_s);

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return apollo::common::util::StrCat("path_data:", path_data_.DebugString(),
                                      "speed_data:", speed_data_.DebugString());
}

void ReferenceLineInfo::ExportTurnSignal(VehicleSignal* signal) const {
  // set vehicle change lane signal
  CHECK_NOTNULL(signal);

  signal->Clear();
  signal->set_turn_signal(VehicleSignal::TURN_NONE);
  if (IsChangeLanePath()) {
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {
      signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    return;
  }
  // check lane's turn type
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      break;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {
      signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      break;
    } else if (turn == hdmap::Lane::U_TURN) {
      // check left or right by geometry.
      auto start_xy =
          common::util::MakeVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = common::util::MakeVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) / 2.0));
      auto end_xy =
          common::util::MakeVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      break;
    }
  }
}

bool ReferenceLineInfo::IsRightTurnPath() const {
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  constexpr double kRightTurnStartBuff = 1.0;
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + kRightTurnStartBuff) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::RIGHT_TURN) {
      return true;
    }
  }
  return false;
}

bool ReferenceLineInfo::ReachedDestination() const {
  constexpr double kDestinationDeltaS = 0.05;
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return false;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return false;
  }
  if (!reference_line_.IsOnRoad(
          dest_ptr->obstacle()->PerceptionBoundingBox().center())) {
    return false;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return adc_sl_boundary_.end_s() + kDestinationDeltaS > stop_s;
}

void ReferenceLineInfo::ExportDecision(DecisionResult* decision_result) const {
  MakeDecision(decision_result);
  ExportTurnSignal(decision_result->mutable_vehicle_signal());
  auto* main_decision = decision_result->mutable_main_decision();
  if (main_decision->has_stop()) {
    main_decision->mutable_stop()->set_change_lane_type(
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}

void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result) const {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();

  // cruise by default
  decision_result->mutable_main_decision()->mutable_cruise();

  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result);
  SetObjectDecisions(decision_result->mutable_object_decision());
}

void ReferenceLineInfo::MakeMainMissionCompleteDecision(
    DecisionResult* decision_result) const {
  if (!decision_result->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION) {
    return;
  }
  const auto& adc_pos = AdcPlanningPoint().path_point();
  if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >
      FLAGS_destination_check_distance) {
    return;
  }
  if (ReachedDestination()) {
    return;
  }
  auto mission_complete =
      decision_result->mutable_main_decision()->mutable_mission_complete();
  mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
  mission_complete->set_stop_heading(main_stop.stop_heading());
}

int ReferenceLineInfo::MakeMainStopDecision(
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {
    const auto& obstacle = path_obstacle->obstacle();
    const auto& object_decision = path_obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]";
      continue;
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}

void ReferenceLineInfo::SetObjectDecisions(
    ObjectDecisions* object_decisions) const {
  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {
    if (!path_obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();

    const auto& obstacle = path_obstacle->obstacle();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (path_obstacle->HasLateralDecision() &&
        !path_obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          path_obstacle->LateralDecision());
    }
    if (path_obstacle->HasLongitudinalDecision() &&
        !path_obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          path_obstacle->LongitudinalDecision());
    }
  }
}

void ReferenceLineInfo::ExportEngageAdvice(EngageAdvice* engage_advice) const {
  constexpr double kMaxAngleDiff = M_PI / 6.0;
  auto* prev_advice = GetPlanningStatus()->mutable_engage_advice();
  if (!prev_advice->has_advice()) {
    prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
  }
  if (!IsDrivable()) {
    if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
    } else {
      prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
    prev_advice->set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {
    if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
    } else {
      prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
    prev_advice->set_reason("Not on reference line");
  } else {
    // check heading
    auto ref_point =
        reference_line_.GetReferencePoint(adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_.heading(), ref_point.heading()) >
        kMaxAngleDiff) {
      if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
        prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
      } else {
        prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
      }
      prev_advice->set_reason("Vehicle heading is not aligned");
    } else {
      if (vehicle_state_.driving_mode() !=
          Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
        prev_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      } else {
        prev_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      }
      prev_advice->clear_reason();
    }
  }
  engage_advice->CopyFrom(*prev_advice);
}

void ReferenceLineInfo::MakeEStopDecision(
    DecisionResult* decision_result) const {
  decision_result->Clear();

  MainEmergencyStop* main_estop =
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions =
      decision_result->mutable_object_decision();
  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {
    auto* object_decision = object_decisions->add_decision();
    const auto& obstacle = path_obstacle->obstacle();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}
}  // namespace planning
}  // namespace apollo
