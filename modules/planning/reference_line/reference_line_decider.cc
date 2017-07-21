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

using ErrorCode = apollo::common::ErrorCode;

ReferenceLineDecider::ReferenceLineDecider() { _it = _reference_lines.begin(); }

ErrorCode ReferenceLineDecider::init(const DataCenter& data_center) {
  _reference_lines.clear();
  const auto& routing =
      data_center.current_frame()->environment().routing_proxy().routing();
  const auto& state = data_center.current_frame()
                          ->environment()
                          .vehicle_state_proxy()
                          .vehicle_state();
  auto* master = data_center.mutable_master();

  // TODO: call build_reference_lines(data_center, router) when new routing are
  // used!
  if (routing.header().sequence_num() != _last_route_sequence_num ||
      Double::compare(routing.header().timestamp_sec(),
                      _last_route_timestamp) != 0) {
    // now, get a new routing, then do initializing.
    _route_reference_lines.clear();
    std::vector<apollo::hdmap::RoutingResult> lanes;
    const auto& map = data_center.map();
    ErrorCode ret =
        data_center.current_frame()->environment().routing_proxy().lanes(
            map, &lanes);
    if (ret != ErrorCode::OK) {
      AERROR << "Fail to initialize lanes.";
      return ret;
    }
    for (const auto& lane : lanes) {
      _route_reference_lines.emplace_back();
      ret = map.get_reference_line_from_routing(
          lane, 0.0, lane.measurement().distance(),
          &(_route_reference_lines.back()));
      if (ret != ErrorCode::OK) {
        AERROR << "Fail to initialize reference line.";
        return ret;
      }
    }
    _last_route_sequence_num = routing.header().sequence_num();
    _last_route_timestamp = routing.header().timestamp_sec();
    _current_route_index = 0;
    _current_s = 0.0;
    master->set_state(MasterStateMachine::MasterState::CRUISE);
  }
  // Logic as follow :
  // 0. check the state of master:
  if (_route_reference_lines.empty()) {
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
  Eigen::Vector2d location(state.pose().position().x(),
                           state.pose().position().y());
  std::size_t next_route_index = _current_route_index;
  for (std::size_t i = _current_route_index;
       i < _route_reference_lines.size() && i - _current_route_index < 2; ++i) {
    sl_points.emplace_back();
    if (!_route_reference_lines[i].get_point_in_Frenet_frame(
            location, &sl_points.back())) {
      sl_points.pop_back();
    } else {
      if (fabs(sl_points.back().l()) <
          _route_reference_lines[i].get_lane_width(sl_points.back().s())) {
        next_route_index = i;
      }
    }
  }

  if (sl_points.empty()) {
    AERROR << "Can not find location in the reference line.";
    return ErrorCode::PLANNING_ERROR;
  }
  if (_current_route_index != next_route_index) {
    master->set_state(MasterStateMachine::MasterState::CRUISE);
    _current_s = sl_points.back().s();
  } else {
    _current_s = std::max(sl_points.begin()->s(), _current_s);
  }

  _current_route_index = next_route_index;

  // 3. put current reference line in reference_lines.
  auto reference_line = new ReferenceLine();
  reference_line->move(_route_reference_lines[_current_route_index]);
  _reference_lines.emplace_back(reference_line);

  // 4. judge if the s in the range of lane change.
  next_route_index = _current_route_index + 1;
  if (next_route_index < _route_reference_lines.size()) {
    common::SLPoint sl_point;
    if (_route_reference_lines[next_route_index].get_point_in_Frenet_frame(
            location, &sl_point)) {
      // 5. if yes, put the next reference lane in the front of reference.
      reference_line = new ReferenceLine();
      reference_line->move(_route_reference_lines[next_route_index]);
      _reference_lines.emplace_front(reference_line);
    }
  }

  // 6. if finished, then change state to finsh.
  if (next_route_index == _route_reference_lines.size()) {
    if (_current_s + 4.5 >=
        _route_reference_lines.back().reference_map_line().length()) {
      master->set_state(MasterStateMachine::MasterState::FINISH);
    }
  }
  _it = _reference_lines.begin();
  return ErrorCode::OK;
}

ErrorCode ReferenceLineDecider::build_reference_lines(
    const DataCenter& data_center,
    const apollo::hdmap::RoutingResult& routing) {
  auto* master = data_center.mutable_master();
  if (routing.header().sequence_num() != _last_route_sequence_num ||
      Double::compare(routing.header().timestamp_sec(),
                      _last_route_timestamp) != 0) {
    // now, get a new routing, then do initializing.
    _route_reference_lines.clear();
    std::vector<apollo::hdmap::RoutingResult> lanes;
    const auto& map = data_center.map();
    ErrorCode ret =
        data_center.current_frame()->environment().routing_proxy().lanes(
            map, &lanes);
    if (ret != ErrorCode::OK) {
      AERROR << "Could not initial lanes.";
      return ret;
    }
    for (const auto& lane : lanes) {
      _route_reference_lines.emplace_back();
      ret = map.get_reference_line_from_routing(
          lane, 0.0, lane.measurement().distance(),
          &(_route_reference_lines.back()));
      if (ret != ErrorCode::OK) {
        AERROR << "Could not initial reference line.";
        return ret;
      }
    }
    _last_route_sequence_num = routing.header().sequence_num();
    _last_route_timestamp = routing.header().timestamp_sec();
    _current_route_index = 0;
    _current_s = 0.0;
    master->set_state(MasterStateMachine::MasterState::CRUISE);
  }
  return ErrorCode::OK;
}

std::unique_ptr<ReferenceLine> ReferenceLineDecider::next_reference_line() {
  std::unique_ptr<ReferenceLine> ret = nullptr;
  if (_it != _reference_lines.end()) {
    ret = std::move(*_it);
    ++_it;
  }
  return ret;
}

bool ReferenceLineDecider::has_next() const {
  return _it != _reference_lines.end();
}

std::string ReferenceLineDecider::to_json() const { return ""; }

std::size_t ReferenceLineDecider::num_of_reference_lines() const {
  return _reference_lines.size();
}

}  // namespace planning
}  // namespace apollo
