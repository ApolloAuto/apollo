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

#include "modules/routing/core/node_range_manager.h"

#include "modules/routing/graph/topo_node.h"

namespace apollo {
namespace routing {

void NodeRangeManager::init_node_range(double start_node_s, double end_node_s,
                                       const TopoNode* start_node,
                                       const TopoNode* end_node) {
  _range_map.clear();
  init_in_neighbor(start_node, start_node_s, start_node->length());
  init_out_neighbor(start_node, start_node_s, start_node->length());

  init_in_neighbor(end_node, 0.0, end_node_s);
  init_out_neighbor(end_node, 0.0, end_node_s);
}

NodeRange NodeRangeManager::get_node_range(const TopoNode* topo_node) const {
  const auto& iter = _range_map.find(topo_node);
  if (iter != _range_map.end()) {
    return iter->second;
  }
  return NodeRange(0.0, topo_node->length());
}

double NodeRangeManager::get_node_start_s(const TopoNode* topo_node) const {
  return get_node_range(topo_node).start_s;
}

double NodeRangeManager::get_node_end_s(const TopoNode* topo_node) const {
  return get_node_range(topo_node).end_s;
}

void NodeRangeManager::init_in_neighbor(const TopoNode* cur_node,
                                        double start_s, double end_s) {
  for (const auto* edge : cur_node->in_from_left_or_right_edge()) {
    const auto* from_node = edge->from_node();
    if (_range_map.count(from_node) != 0) {
      // in case of start and end in the same lane
      auto& range = _range_map[from_node];
      range.start_s = std::max(range.start_s, start_s);
      range.end_s = std::min(range.end_s, end_s);
    } else {
      auto& range = _range_map[from_node];
      range.start_s = start_s;
      range.end_s = end_s;
      init_in_neighbor(from_node, start_s, end_s);
    }
  }
}

void NodeRangeManager::init_out_neighbor(const TopoNode* cur_node,
                                         double start_s, double end_s) {
  for (const auto* edge : cur_node->out_to_left_or_right_edge()) {
    const auto* to_node = edge->to_node();
    if (_range_map.count(to_node) != 0) {
      // in case of start and end in the same lane
      auto& range = _range_map[to_node];
      range.start_s = std::max(range.start_s, start_s);
      range.end_s = std::min(range.end_s, end_s);
    } else {
      auto& range = _range_map[to_node];
      range.start_s = start_s;
      range.end_s = end_s;
      init_out_neighbor(to_node, start_s, end_s);
    }
  }
}

}  // namespace routing
}  // namespace apollo
