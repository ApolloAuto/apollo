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
 * @file reference_line_decider.cc
 **/
#include "modules/planning/reference_line/reference_line_decider.h"

#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using ErrorCode = common::ErrorCode;

ReferenceLineDecider::ReferenceLineDecider() { it_ = reference_lines_.begin(); }

ErrorCode ReferenceLineDecider::init(const DataCenter& data_center) {
  reference_lines_.clear();
  const auto& routing =
      data_center.current_frame()->environment().routing_proxy().routing();
  const auto& state = data_center.current_frame()
                          ->environment()
                          .vehicle_state_proxy()
                          .vehicle_state();
  auto* master = data_center.mutable_master();

  // TODO: call build_reference_lines(data_center, router) when new routing are
  // used!
  if (routing.header().sequence_num() != last_route_sequence_num_ ||
      Double::compare(routing.header().timestamp_sec(),
                      last_route_timestamp_) != 0) {
    // now, get a new routing, then do initializing.
    route_reference_lines_.clear();
    std::vector<hdmap::RoutingResult> lanes;
    const auto& map = data_center.map();
    ErrorCode ret =
        data_center.current_frame()->environment().routing_proxy().lanes(
            map, &lanes);
    if (ret != ErrorCode::OK) {
      AERROR << "Fail to initialize lanes.";
      return ret;
    }
    for (const auto& lane : lanes) {
      route_reference_lines_.emplace_back();
      ret = map.get_reference_line_from_routing(
          lane, 0.0, lane.measurement().distance(),
          &(route_reference_lines_.back()));
      if (ret != ErrorCode::OK) {
        AERROR << "Fail to initialize reference line.";
        return ret;
      }
    }
    last_route_sequence_num_ = routing.header().sequence_num();
    last_route_timestamp_ = routing.header().timestamp_sec();
    current_route_index_ = 0;
    current_s_ = 0.0;
    master->set_state(MasterStateMachine::MasterState::CRUISE);
  }
  // Logic as follow :
  // 0. check the state of master:
  if (route_reference_lines_.empty()) {
    AERROR << "Have not got reference line yet.";
    return ErrorCode::PLANNING_ERROR;
  }

  if (master->state() != MasterStateMachine::MasterState::CRUISE &&
      master->state() != MasterStateMachine::MasterState::CHANGING_LANE) {
    AERROR << "Only cruise or lane_chaning is accpeted.";
    return ErrorCode::PLANNING_ERROR;
  }

  // 1. from current_route_index to size of route_reference_line, calculate the
  // sl point.
  std::vector<SLPoint> sl_points;
  common::math::Vec2d location(state.pose().position().x(),
                           state.pose().position().y());
  std::uint32_t next_route_index = current_route_index_;
  for (std::uint32_t i = current_route_index_;
       i < route_reference_lines_.size() && i - current_route_index_ < 2; ++i) {
    sl_points.emplace_back();
    if (!route_reference_lines_[i].get_point_in_Frenet_frame(
            location, &sl_points.back())) {
      sl_points.pop_back();
    } else {
      if (fabs(sl_points.back().l()) <
          route_reference_lines_[i].get_lane_width(sl_points.back().s())) {
        next_route_index = i;
      }
    }
  }

  if (sl_points.empty()) {
    AERROR << "Can not find location in the reference line.";
    return ErrorCode::PLANNING_ERROR;
  }
  if (current_route_index_ != next_route_index) {
    master->set_state(MasterStateMachine::MasterState::CRUISE);
    current_s_ = sl_points.back().s();
  } else {
    current_s_ = std::max(sl_points.begin()->s(), current_s_);
  }

  current_route_index_ = next_route_index;

  // 3. put current reference line in reference_lines.
  auto reference_line = new ReferenceLine();
  reference_line->move(route_reference_lines_[current_route_index_]);
  reference_lines_.emplace_back(reference_line);

  // 4. judge if the s in the range of lane change.
  next_route_index = current_route_index_ + 1;
  if (next_route_index < route_reference_lines_.size()) {
    common::SLPoint sl_point;
    if (route_reference_lines_[next_route_index].get_point_in_Frenet_frame(
            location, &sl_point)) {
      // 5. if yes, put the next reference lane in the front of reference.
      reference_line = new ReferenceLine();
      reference_line->move(route_reference_lines_[next_route_index]);
      reference_lines_.emplace_front(reference_line);
    }
  }

  // 6. if finished, then change state to finsh.
  if (next_route_index == route_reference_lines_.size()) {
    if (current_s_ + 4.5 >=
        route_reference_lines_.back().reference_map_line().length()) {
      master->set_state(MasterStateMachine::MasterState::FINISH);
    }
  }
  it_ = reference_lines_.begin();
  return ErrorCode::OK;
}

ErrorCode ReferenceLineDecider::build_reference_lines(
    const DataCenter& data_center, const hdmap::RoutingResult& routing) {
  auto* master = data_center.mutable_master();
  if (routing.header().sequence_num() != last_route_sequence_num_ ||
      Double::compare(routing.header().timestamp_sec(),
                      last_route_timestamp_) != 0) {
    // now, get a new routing, then do initializing.
    route_reference_lines_.clear();
    std::vector<hdmap::RoutingResult> lanes;
    const auto& map = data_center.map();
    ErrorCode ret =
        data_center.current_frame()->environment().routing_proxy().lanes(
            map, &lanes);
    if (ret != ErrorCode::OK) {
      AERROR << "Could not initial lanes.";
      return ret;
    }
    for (const auto& lane : lanes) {
      route_reference_lines_.emplace_back();
      ret = map.get_reference_line_from_routing(
          lane, 0.0, lane.measurement().distance(),
          &(route_reference_lines_.back()));
      if (ret != ErrorCode::OK) {
        AERROR << "Could not initial reference line.";
        return ret;
      }
    }
    last_route_sequence_num_ = routing.header().sequence_num();
    last_route_timestamp_ = routing.header().timestamp_sec();
    current_route_index_ = 0;
    current_s_ = 0.0;
    master->set_state(MasterStateMachine::MasterState::CRUISE);
  }
  return ErrorCode::OK;
}

std::unique_ptr<ReferenceLine> ReferenceLineDecider::next_reference_line() {
  std::unique_ptr<ReferenceLine> ret = nullptr;
  if (it_ != reference_lines_.end()) {
    ret = std::move(*it_);
    ++it_;
  }
  return ret;
}

bool ReferenceLineDecider::has_next() const {
  return it_ != reference_lines_.end();
}

std::string ReferenceLineDecider::to_json() const { return ""; }

std::uint32_t ReferenceLineDecider::num_of_reference_lines() const {
  return reference_lines_.size();
}

}  // namespace planning
}  // namespace apollo
