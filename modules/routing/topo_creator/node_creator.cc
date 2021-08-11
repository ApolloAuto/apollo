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

#include "modules/routing/topo_creator/node_creator.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace routing {
namespace node_creator {

namespace {

using ::google::protobuf::RepeatedPtrField;

using apollo::hdmap::Lane;
using apollo::hdmap::LaneBoundary;
using apollo::hdmap::LaneBoundaryType;

bool IsAllowedOut(const LaneBoundaryType& type) {
  if (type.types(0) == LaneBoundaryType::DOTTED_YELLOW ||
      type.types(0) == LaneBoundaryType::DOTTED_WHITE) {
    return true;
  }
  return false;
}

double GetLengthbyRate(double cur_s, double cur_total_length,
                       double target_length) {
  double new_length = cur_s / cur_total_length * target_length;
  return std::min(new_length, target_length);
}

double GetLaneLength(const Lane& lane) {
  double length = 0.0;
  for (const auto& segment : lane.central_curve().segment()) {
    length += segment.length();
  }
  return length;
}

void AddOutBoundary(const LaneBoundary& bound, double lane_length,
                    RepeatedPtrField<CurveRange>* const out_range) {
  for (int i = 0; i < bound.boundary_type_size(); ++i) {
    if (!IsAllowedOut(bound.boundary_type(i))) {
      continue;
    }
    CurveRange* range = out_range->Add();
    range->mutable_start()->set_s(GetLengthbyRate(bound.boundary_type(i).s(),
                                                  bound.length(), lane_length));
    if (i != bound.boundary_type_size() - 1) {
      range->mutable_end()->set_s(GetLengthbyRate(
          bound.boundary_type(i + 1).s(), bound.length(), lane_length));
    } else {
      range->mutable_end()->set_s(lane_length);
    }
  }
}

void InitNodeInfo(const Lane& lane, const std::string& road_id,
                  Node* const node) {
  double lane_length = GetLaneLength(lane);
  node->set_lane_id(lane.id().id());
  node->set_road_id(road_id);
  AddOutBoundary(lane.left_boundary(), lane_length, node->mutable_left_out());
  AddOutBoundary(lane.right_boundary(), lane_length, node->mutable_right_out());
  node->set_length(lane_length);
  node->mutable_central_curve()->CopyFrom(lane.central_curve());
  node->set_is_virtual(true);
  if (!lane.has_junction_id() ||
      lane.left_neighbor_forward_lane_id_size() > 0 ||
      lane.right_neighbor_forward_lane_id_size() > 0) {
    node->set_is_virtual(false);
  }
}

void InitNodeCost(const Lane& lane, const RoutingConfig& routing_config,
                  Node* const node) {
  double lane_length = GetLaneLength(lane);
  double speed_limit =
      lane.has_speed_limit() ? lane.speed_limit() : routing_config.base_speed();
  double ratio = speed_limit >= routing_config.base_speed()
                     ? std::sqrt(routing_config.base_speed() / speed_limit)
                     : 1.0;
  double cost = lane_length * ratio;
  if (lane.has_turn()) {
    if (lane.turn() == Lane::LEFT_TURN) {
      cost += routing_config.left_turn_penalty();
    } else if (lane.turn() == Lane::RIGHT_TURN) {
      cost += routing_config.right_turn_penalty();
    } else if (lane.turn() == Lane::U_TURN) {
      cost += routing_config.uturn_penalty();
    }
  }
  node->set_cost(cost);
}

}  // namespace

void GetPbNode(const hdmap::Lane& lane, const std::string& road_id,
               const RoutingConfig& routingconfig, Node* const node) {
  InitNodeInfo(lane, road_id, node);
  InitNodeCost(lane, routingconfig, node);
}

}  // namespace node_creator
}  // namespace routing
}  // namespace apollo
