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

#include "modules/routing/topo_creator/edge_creator.h"

#include <cmath>

namespace apollo {
namespace routing {
namespace edge_creator {

void GetPbEdge(const Node& node_from, const Node& node_to,
               const Edge::DirectionType& type,
               const RoutingConfig& routing_config, Edge* edge) {
  edge->set_from_lane_id(node_from.lane_id());
  edge->set_to_lane_id(node_to.lane_id());
  edge->set_direction_type(type);

  edge->set_cost(0.0);
  if (type == Edge::LEFT || type == Edge::RIGHT) {
    const auto& target_range =
        (type == Edge::LEFT) ? node_from.left_out() : node_from.right_out();
    double changing_area_length = 0.0;
    for (const auto& range : target_range) {
      changing_area_length += range.end().s() - range.start().s();
    }
    double ratio = 1.0;
    if (changing_area_length < routing_config.base_changing_length()) {
      ratio = std::pow(
          changing_area_length / routing_config.base_changing_length(), -1.5);
    }
    edge->set_cost(routing_config.change_penalty() * ratio);
  }
}

}  // namespace edge_creator
}  // namespace routing
}  // namespace apollo
