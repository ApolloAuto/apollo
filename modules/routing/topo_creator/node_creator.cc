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

#include "topo_creator/node_creator.h"

#include <math.h>

namespace adu {
namespace routing {

namespace {

using ::google::protobuf::RepeatedPtrField;

using ::adu::common::hdmap::Lane;
using ::adu::common::hdmap::LaneBoundary;
using ::adu::common::hdmap::LaneBoundaryType;

using ::adu::routing::common::Node;
using ::adu::routing::common::Edge;
using ::adu::routing::common::CurveRange;

const double BASE_SPEED = 4.167;       // 15kmh
const double LEFT_TURN_PENALTY = 50;   // meter
const double RIGHT_TURN_PENALTY = 20;  // meter
const double UTURN_PENALTY = 100;      // meter

bool is_allowed_out(const LaneBoundaryType& type) {
  if (type.types(0) == LaneBoundaryType::DOTTED_YELLOW ||
      type.types(0) == LaneBoundaryType::DOTTED_WHITE) {
    return true;
  }
  return false;
}

double get_length_by_rate(double cur_s, double cur_total_length,
                          double target_length) {
  double new_length = cur_s / cur_total_length * target_length;
  return std::min(new_length, target_length);
}

double get_lane_length(const Lane& lane) {
  double length = 0.0;
  for (const auto& segment : lane.central_curve().segment()) {
    length += segment.length();
  }
  return length;
}

}  // namespace

void NodeCreator::get_pb_node(const Lane& lane, const std::string& road_id,
                              Node* pb_node) {
  init_node_info(lane, road_id, pb_node);
  init_node_cost(lane, pb_node);
}

void NodeCreator::add_out_boundary(
    const LaneBoundary& bound, double lane_length,
    RepeatedPtrField<CurveRange>* const out_range) {
  for (int i = 0; i < bound.boundary_type_size(); ++i) {
    if (!is_allowed_out(bound.boundary_type(i))) {
      continue;
    }
    CurveRange* range = out_range->Add();
    range->mutable_start()->set_s(get_length_by_rate(
        bound.boundary_type(i).s(), bound.length(), lane_length));
    if (i != bound.boundary_type_size() - 1) {
      range->mutable_end()->set_s(get_length_by_rate(
          bound.boundary_type(i + 1).s(), bound.length(), lane_length));
    } else {
      range->mutable_end()->set_s(lane_length);
    }
  }
}

void NodeCreator::init_node_info(const Lane& lane, const std::string& road_id,
                                 Node* const node) {
  double lane_length = get_lane_length(lane);
  node->set_lane_id(lane.id().id());
  node->set_road_id(road_id);
  add_out_boundary(lane.left_boundary(), lane_length, node->mutable_left_out());
  add_out_boundary(lane.right_boundary(), lane_length,
                   node->mutable_right_out());
  node->set_length(lane_length);
  node->mutable_central_curve()->CopyFrom(lane.central_curve());
  node->set_is_virtual(true);
  if (!lane.has_junction_id() ||
      lane.left_neighbor_forward_lane_id_size() > 0 ||
      lane.right_neighbor_forward_lane_id_size() > 0) {
    node->set_is_virtual(false);
  }
}

void NodeCreator::init_node_cost(const Lane& lane, Node* const node) {
  double lane_length = get_lane_length(lane);
  double speed_limit =
      (lane.has_speed_limit()) ? lane.speed_limit() : BASE_SPEED;
  double ratio =
      (speed_limit >= BASE_SPEED) ? (1 / sqrt(speed_limit / BASE_SPEED)) : 1.0;
  double cost = lane_length * ratio;
  if (lane.has_turn()) {
    if (lane.turn() == ::adu::common::hdmap::Lane::LEFT_TURN) {
      cost += LEFT_TURN_PENALTY;
    } else if (lane.turn() == ::adu::common::hdmap::Lane::RIGHT_TURN) {
      cost += RIGHT_TURN_PENALTY;
    } else if (lane.turn() == ::adu::common::hdmap::Lane::U_TURN) {
      cost += UTURN_PENALTY;
    }
  }
  node->set_cost(cost);
}

}  // namespace routing
}  // namespace adu
