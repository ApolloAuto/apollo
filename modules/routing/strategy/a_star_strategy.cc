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

#include <algorithm>
#include <fstream>
#include <limits>
#include <limits>
#include <queue>

#include "ros/ros.h"

#include "graph/topo_edge.h"
#include "graph/topo_graph.h"
#include "graph/topo_node.h"
#include "strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

struct SearchNode {
  const TopoNode* topo_node;
  double f;

  SearchNode() : topo_node(nullptr), f(std::numeric_limits<double>::max()) {}
  SearchNode(const TopoNode* node)
      : topo_node(node), f(std::numeric_limits<double>::max()) {}
  SearchNode(const SearchNode& search_node)
      : topo_node(search_node.topo_node), f(search_node.f) {}

  bool operator<(const SearchNode& node) const {
    // in order to let the top of preority queue is the smallest one!
    return f > node.f;
  }

  bool operator==(const SearchNode& node) const {
    return topo_node == node.topo_node;
  }
};

double get_cost_to_neighbor(const TopoEdge* edge) {
  return (edge->cost() + edge->to_node()->cost());
}

bool reconstruct(
    const std::unordered_map<const TopoNode*, const TopoNode*>& came_from,
    const TopoNode* dest_node, std::vector<const TopoNode*>* result_nodes) {
  result_nodes->clear();
  const TopoNode* cur_node = dest_node;
  result_nodes->push_back(cur_node);
  auto iter = came_from.find(cur_node);
  while (iter != came_from.end()) {
    cur_node = iter->second;
    result_nodes->push_back(cur_node);
    iter = came_from.find(cur_node);
  }
  std::reverse(result_nodes->begin(), result_nodes->end());
  return true;
}

}  // namespace

void AStarStrategy::clear() {
  _closed_set.clear();
  _open_set.clear();
  _came_from.clear();
  _g_score.clear();
}

double AStarStrategy::heuristic_cost(const TopoNode* src_node,
                                     const TopoNode* dest_node) {
  const ::adu::common::hdmap::Point& src_point = src_node->anchor_point();
  const ::adu::common::hdmap::Point& dest_point = dest_node->anchor_point();
  double distance = sqrt(pow(src_point.x() - dest_point.x(), 2) +
                         pow(src_point.y() - dest_point.y(), 2) +
                         pow(src_point.z() - dest_point.z(), 2));
  return distance;
}

bool AStarStrategy::search(
    const TopoGraph* graph, const TopoNode* src_node, const TopoNode* dest_node,
    const std::unordered_set<const TopoNode*>& black_list,
    std::vector<const TopoNode*>* const result_nodes) {
  clear();
  ROS_INFO("Start A* search algorithm.");

  _closed_set.insert(black_list.begin(), black_list.end());

  std::priority_queue<SearchNode> open_set_detail;

  SearchNode src_search_node(src_node);
  src_search_node.f = heuristic_cost(src_node, dest_node);
  open_set_detail.push(src_search_node);

  _open_set.insert(src_node);
  _g_score[src_node] = 0.0;

  SearchNode current_node;
  while (!open_set_detail.empty()) {
    current_node = open_set_detail.top();
    if (current_node == dest_node) {
      if (!reconstruct(_came_from, current_node.topo_node, result_nodes)) {
        ROS_ERROR("Failed to reconstruct route.");
        return false;
      }
      return true;
    }
    _open_set.erase(current_node.topo_node);
    open_set_detail.pop();

    if (_closed_set.count(current_node.topo_node) != 0) {
      // if showed before, just skip...
      continue;
    }
    _closed_set.emplace(current_node.topo_node);

    double tentative_g_score = 0.0;
    for (const auto* edge : current_node.topo_node->out_to_all_edge()) {
      const auto* to_node = edge->to_node();
      if (_closed_set.count(to_node) == 1) {
        continue;
      }
      tentative_g_score =
          _g_score[current_node.topo_node] + get_cost_to_neighbor(edge);
      if (_open_set.count(to_node) != 0 &&
          tentative_g_score >= _g_score[to_node]) {
        continue;
      }
      _g_score[to_node] = tentative_g_score;
      SearchNode next_node(to_node);
      next_node.f =
          tentative_g_score + heuristic_cost(current_node.topo_node, to_node);
      open_set_detail.push(next_node);
      _came_from[to_node] = current_node.topo_node;

      if (_open_set.count(to_node) == 0) {
        _open_set.insert(to_node);
      }
    }
  }
  ROS_ERROR("Failed to find goal lane with id %s",
            dest_node->lane_id().c_str());
  return false;
}

}  // namespace routing
}  // namespace apollo
