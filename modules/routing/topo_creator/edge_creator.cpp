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

#include "topo_creator/edge_creator.h"

#include <math.h>

namespace adu {
namespace routing {

namespace {

using ::adu::routing::common::Node;
using ::adu::routing::common::Edge;
using ::adu::routing::common::CurveRange;

const double CHANGE_PENALTY = 50; // equal to 50 meter length
const double BASE_CHANGING_LENGTH = 50;

} // namespace

void EdgeCreator::get_pb_edge(const Node& node_from,
                              const Node& node_to,
                              const Edge::DirectionType& type,
                              Edge* pb_edge) {
    init_edge_info(node_from, node_to, type, pb_edge);
    init_edge_cost(node_from, node_to, type, pb_edge);
}

void EdgeCreator::init_edge_info(const Node& node_from,
                                 const Node& node_to,
                                 const Edge::DirectionType& type,
                                 Edge* const edge) {
    edge->set_from_lane_id(node_from.lane_id());
    edge->set_to_lane_id(node_to.lane_id());
    edge->set_direction_type(type);
}

void EdgeCreator::init_edge_cost(const Node& node_from,
                                 const Node& node_to,
                                 const Edge::DirectionType& type,
                                 Edge* const edge) {
    edge->set_cost(0.0);
    if (type == Edge::LEFT || type == Edge::RIGHT) {
        const auto& target_range =
                (type == Edge::LEFT) ? node_from.left_out() : node_to.right_out();
        double changing_area_length = 0.0;
        for (const auto& range : target_range) {
            changing_area_length += range.end().s() - range.start().s();
        }
        double ratio = (changing_area_length >= BASE_CHANGING_LENGTH) ?
                        pow(changing_area_length / BASE_CHANGING_LENGTH , -1.5) : 1.0;
        edge->set_cost(CHANGE_PENALTY * ratio);
    }
}

} // namespace routing
} // namespace adu

