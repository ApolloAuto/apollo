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
#include <queue>

#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_node.h"
#include "modules/routing/strategy/a_star_strategy.h"

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

double GetCostToNeighbor(const TopoEdge* edge) {
  return (edge->Cost() + edge->ToNode()->Cost());
}

bool ReConstruct(
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

void AStarStrategy::Clear() {
  closed_set_.clear();
  open_set_.clear();
  came_from_.clear();
  g_score_.clear();
}

double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                    const TopoNode* dest_node) {
  const ::apollo::common::PointENU& src_point = src_node->AnchorPoint();
  const ::apollo::common::PointENU& dest_point = dest_node->AnchorPoint();
  double distance = sqrt(pow(src_point.x() - dest_point.x(), 2) +
                         pow(src_point.y() - dest_point.y(), 2) +
                         pow(src_point.z() - dest_point.z(), 2));
  return distance;
}

bool AStarStrategy::Search(
    const TopoGraph* graph, const TopoNode* src_node, const TopoNode* dest_node,
    const std::unordered_set<const TopoNode*>& black_list,
    std::vector<const TopoNode*>* const result_nodes) {
  Clear();
  AINFO << "Start A* search algorithm.";

  closed_set_.insert(black_list.begin(), black_list.end());

  std::priority_queue<SearchNode> open_set_detail;

  SearchNode src_search_node(src_node);
  src_search_node.f = HeuristicCost(src_node, dest_node);
  open_set_detail.push(src_search_node);

  open_set_.insert(src_node);
  g_score_[src_node] = 0.0;

  SearchNode current_node;
  while (!open_set_detail.empty()) {
    current_node = open_set_detail.top();
    if (current_node == dest_node) {
      if (!ReConstruct(came_from_, current_node.topo_node, result_nodes)) {
        AERROR << "Failed to ReConstruct route.";
        return false;
      }
      return true;
    }
    open_set_.erase(current_node.topo_node);
    open_set_detail.pop();

    if (closed_set_.count(current_node.topo_node) != 0) {
      // if showed before, just skip...
      continue;
    }
    closed_set_.emplace(current_node.topo_node);

    double tentativeg_score_ = 0.0;
    for (const auto* edge : current_node.topo_node->OutToAllEdge()) {
      const auto* to_node = edge->ToNode();
      if (closed_set_.count(to_node) == 1) {
        continue;
      }
      tentativeg_score_ =
          g_score_[current_node.topo_node] + GetCostToNeighbor(edge);
      if (open_set_.count(to_node) != 0 &&
          tentativeg_score_ >= g_score_[to_node]) {
        continue;
      }
      g_score_[to_node] = tentativeg_score_;
      SearchNode next_node(to_node);
      next_node.f =
          tentativeg_score_ + HeuristicCost(current_node.topo_node, to_node);
      open_set_detail.push(next_node);
      came_from_[to_node] = current_node.topo_node;

      if (open_set_.count(to_node) == 0) {
        open_set_.insert(to_node);
      }
    }
  }
  AERROR << "Failed to find goal lane";
  return false;
}

}  // namespace routing
}  // namespace apollo
