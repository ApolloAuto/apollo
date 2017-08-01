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

#include "ros/ros.h"
#include "graph/topo_range_manager.h"

#include "graph/topo_node.h"
#include "graph/topo_edge.h"

namespace adu {
namespace routing {

void NodeSRangeManager::clear() {
    _range_map.clear();
    _is_start_inited.clear();
    _is_end_inited.clear();
}

void NodeSRangeManager::init_node_range(double start_node_s,
                                        double end_node_s,
                                        const TopoNode* start_node,
                                        const TopoNode* end_node) {
    auto& start_range = _range_map[start_node];
    start_range.set_range_s(start_node_s, start_node->length());
    _is_start_inited.emplace(start_node);
    // if start and end are same node
    if (_range_map.count(end_node) != 0) {
        auto& end_range = _range_map[end_node];
        end_range.set_end_s(std::min(end_range.end_s(), end_node_s));
    } else {
        auto& end_range = _range_map[end_node];
        end_range.set_range_s(0.0, end_node_s);
    }
    _is_end_inited.emplace(end_node);

    init_out_start(start_node, start_node_s);
    init_in_end(end_node, end_node_s);
}

NodeSRange NodeSRangeManager::get_node_range(const TopoNode* topo_node) const {
    const auto& iter = _range_map.find(topo_node);
    if (iter != _range_map.end()) {
        return iter->second;
    }
    return NodeSRange(0.0, topo_node->length());
}

double NodeSRangeManager::get_node_start_s(const TopoNode* topo_node) const {
    return get_node_range(topo_node).start_s();
}

double NodeSRangeManager::get_node_end_s(const TopoNode* topo_node) const {
    return get_node_range(topo_node).end_s();
}

void NodeSRangeManager::init_in_end(const TopoNode* cur_node, double end_s) {
    for (const auto* edge : cur_node->in_from_left_or_right_edge()) {
        const auto* from_node = edge->from_node();
        if (_is_end_inited.find(from_node) != _is_end_inited.end()) {
            continue;
        }
        if (_range_map.count(from_node) != 0) {
            auto& range = _range_map[from_node];
            range.set_end_s(std::min(range.end_s(), end_s));
        } else {
            auto& range = _range_map[from_node];
            range.set_range_s(0.0, end_s);
        }
        _is_end_inited.emplace(from_node);
        init_in_end(from_node, end_s);
    }
}

void NodeSRangeManager::init_out_start(const TopoNode* cur_node, double start_s) {
    for (const auto* edge : cur_node->out_to_left_or_right_edge()) {
        const auto* to_node = edge->to_node();
        if (_is_start_inited.find(to_node) != _is_start_inited.end()) {
            continue;
        }
        ROS_INFO("init out start, id: %s, to id: %s",
                 cur_node->lane_id().c_str(),
                 to_node->lane_id().c_str());
        if (_range_map.count(to_node) != 0) {
            auto& range = _range_map[to_node];
            range.set_start_s(std::max(range.start_s(), start_s));
        } else {
            auto& range = _range_map[to_node];
            range.set_range_s(start_s, to_node->length());
        }
        _is_start_inited.emplace(to_node);
        init_out_start(to_node, start_s);
    }
}

void NodeSRangeManager::init_in_neighbor(const TopoNode* cur_node, double start_s, double end_s) {
    for (const auto* edge : cur_node->in_from_left_or_right_edge()) {
        const auto* from_node = edge->from_node();
        if (_range_map.count(from_node) != 0) {
            // in case of start and end in the same lane
            auto& range = _range_map[from_node];
            range.set_start_s(std::max(range.start_s(), start_s));
            range.set_end_s(std::min(range.end_s(), end_s));
        } else {
            auto& range = _range_map[from_node];
            range.set_start_s(start_s);
            range.set_end_s(end_s);
            init_in_neighbor(from_node, start_s, end_s);
        }
    }
}

void NodeSRangeManager::init_out_neighbor(const TopoNode* cur_node, double start_s, double end_s) {
    for (const auto* edge : cur_node->out_to_left_or_right_edge()) {
        const auto* to_node = edge->to_node();
        if (_range_map.count(to_node) != 0) {
            // in case of start and end in the same lane
            auto& range = _range_map[to_node];
            range.set_start_s(std::max(range.start_s(), start_s));
            range.set_end_s(std::min(range.end_s(), end_s));
        } else {
            auto& range = _range_map[to_node];
            range.set_start_s(start_s);
            range.set_end_s(end_s);
            init_out_neighbor(to_node, start_s, end_s);
        }
    }
}

} // namespace routing
} // namespace adu

