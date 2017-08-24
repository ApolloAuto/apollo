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

void NodeRangeManager::InitNodeRange(double start_node_s, double end_node_s,
                                     const TopoNode* start_node,
                                     const TopoNode* end_node) {
  range_map_.clear();
  if (start_node == end_node) {
    auto& range = range_map_[start_node];
    range.start_s = start_node_s;
    range.end_s = end_node_s;
    InitInNeighbor(start_node, start_node_s, end_node_s);
    InitOutNeighbor(start_node, start_node_s, end_node_s);
  } else {
    InitInNeighbor(start_node, start_node_s, start_node->length());
    InitOutNeighbor(start_node, start_node_s, start_node->length());

    InitInNeighbor(end_node, 0.0, end_node_s);
    InitOutNeighbor(end_node, 0.0, end_node_s);
  }
}

NodeRange NodeRangeManager::GetNodeRange(const TopoNode* topo_node) const {
  const auto& iter = range_map_.find(topo_node);
  if (iter != range_map_.end()) {
    return iter->second;
  }
  return NodeRange(0.0, topo_node->length());
}

void NodeRangeManager::SetNodeS(const TopoNode* topo_node,
                                double node_start_s, double node_end_s) {
  auto& range = range_map_[topo_node];
  range.start_s = node_start_s;
  range.end_s = node_end_s;
}

double NodeRangeManager::GetNodeStartS(const TopoNode* topo_node) const {
  return GetNodeRange(topo_node).start_s;
}

double NodeRangeManager::GetNodeEndS(const TopoNode* topo_node) const {
  return GetNodeRange(topo_node).end_s;
}

void NodeRangeManager::InitInNeighbor(const TopoNode* cur_node,
                                      double start_s, double end_s) {
  for (const auto* edge : cur_node->in_from_left_or_right_edge()) {
    const auto* from_node = edge->from_node();
    if (range_map_.count(from_node) != 0) {
      // in case of start and end in the same lane
      auto& range = range_map_[from_node];
      range.start_s = std::max(range.start_s, start_s);
      range.end_s = std::min(range.end_s, end_s);
    } else {
      auto& range = range_map_[from_node];
      range.start_s = start_s;
      range.end_s = end_s;
      InitInNeighbor(from_node, start_s, end_s);
    }
  }
}

void NodeRangeManager::InitOutNeighbor(const TopoNode* cur_node,
                                       double start_s, double end_s) {
  for (const auto* edge : cur_node->out_to_left_or_right_edge()) {
    const auto* to_node = edge->to_node();
    if (range_map_.count(to_node) != 0) {
      // in case of start and end in the same lane
      auto& range = range_map_[to_node];
      range.start_s = std::max(range.start_s, start_s);
      range.end_s = std::min(range.end_s, end_s);
    } else {
      auto& range = range_map_[to_node];
      range.start_s = start_s;
      range.end_s = end_s;
      InitOutNeighbor(to_node, start_s, end_s);
    }
  }
}

}  // namespace routing
}  // namespace apollo
