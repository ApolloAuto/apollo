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

#include <functional>
#include <memory>
#include <unordered_set>
#include <utility>

#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/decider.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;
using apollo::common::SLPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;

ReferenceLineInfo::ReferenceLineInfo(const hdmap::PncMap* pnc_map,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments,
                                     const TrajectoryPoint& init_adc_point)
    : pnc_map_(pnc_map),
      reference_line_(reference_line),
      init_adc_point_(init_adc_point),
      lanes_(segments) {}

bool ReferenceLineInfo::Init() {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& path_point = init_adc_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0,
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  common::math::Box2d box(center, path_point.theta(), param.length(),
                          param.width());
  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }
  return true;
}

const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}
const common::TrajectoryPoint& ReferenceLineInfo::init_adc_point() const {
  return init_adc_point_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  auto path_obstacle = CreatePathObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path obstacle for " << obstacle->Id();
    return nullptr;
  }
  return path_decision_.AddPathObstacle(*path_obstacle);
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    if (!AddObstacle(obstacle)) {
      AERROR << "Failed to add obstacle " << obstacle->Id();
      return false;
    }
  }
  return true;
}

std::unique_ptr<PathObstacle> ReferenceLineInfo::CreatePathObstacle(
    const Obstacle* obstacle) {
  auto path_obstacle =
      std::unique_ptr<PathObstacle>(new PathObstacle(obstacle));
  if (!path_obstacle->Init(reference_line_, adc_sl_boundary_.end_s())) {
    AERROR << "Failed to create perception sl boundary for obstacle "
           << obstacle->Id();
    return nullptr;
  }
  return path_obstacle;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
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
    const double time_resolution, const double relative_time,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  CHECK(time_resolution > 0.0);
  CHECK(ptr_discretized_trajectory != nullptr);
  if (path_data_.discretized_path().NumOfPoints() == 0) {
    AWARN << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += time_resolution) {
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

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return apollo::common::util::StrCat("path_data:", path_data_.DebugString(),
                                      "speed_data:", speed_data_.DebugString());
}

void ReferenceLineInfo::ExportTurnSignal(VehicleSignal* signal) const {
  // set vehicle change lane signal
  CHECK(signal) << "signal is null";
  signal->Clear();
  signal->set_turn_signal(VehicleSignal::TURN_NONE);
  const auto& next_action = Lanes().NextAction();
  if (next_action != routing::FORWARD) {  // change lane case
    if (next_action == routing::LEFT) {
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (next_action == routing::RIGHT) {
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

void ReferenceLineInfo::ExportDecision(DecisionResult* decision_result) const {
  Decider decider;
  decider.MakeDecision(*this, decision_result);
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

}  // namespace planning
}  // namespace apollo
