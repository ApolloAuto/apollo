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
#include <cmath>
#include <limits>
#include <queue>

#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {
namespace {

struct SearchNode {
  const TopoNode* topo_node = nullptr;
  double f = std::numeric_limits<double>::max();

  SearchNode() = default;
  explicit SearchNode(const TopoNode* node)
      : topo_node(node), f(std::numeric_limits<double>::max()) {}
  SearchNode(const SearchNode& search_node) = default;

  bool operator<(const SearchNode& node) const {
    // in order to let the top of priority queue is the smallest one!
    return f > node.f;
  }

  bool operator==(const SearchNode& node) const {
    return topo_node == node.topo_node;
  }
};

double GetCostToNeighbor(const TopoEdge* edge) {
  return (edge->Cost() + edge->ToNode()->Cost());
}

const TopoNode* GetLargestNode(const std::vector<const TopoNode*>& nodes) {
  double max_range = 0.0;
  const TopoNode* largest = nullptr;
  for (const auto* node : nodes) {
    const double temp_range = node->EndS() - node->StartS();
    if (temp_range > max_range) {
      max_range = temp_range;
      largest = node;
    }
  }
  return largest;
}

bool AdjustLaneChangeBackward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (int i = static_cast<int>(result_node_vec->size()) - 2; i > 0; --i) {
    const auto* from_node = result_node_vec->at(i);
    const auto* to_node = result_node_vec->at(i + 1);
    const auto* base_node = result_node_vec->at(i - 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          from_node->EndS() - from_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(from_node);
      const auto& out_edges = base_node->OutToLeftOrRightEdge();
      for (const auto* edge : out_edges) {
        const auto* candidate_node = edge->ToNode();
        if (candidate_node == from_node) {
          continue;
        }
        if (candidate_node->GetOutEdgeTo(to_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != from_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

bool AdjustLaneChangeForward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (size_t i = 1; i < result_node_vec->size() - 1; ++i) {
    const auto* from_node = result_node_vec->at(i - 1);
    const auto* to_node = result_node_vec->at(i);
    const auto* base_node = result_node_vec->at(i + 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          to_node->EndS() - to_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(to_node);
      const auto& in_edges = base_node->InFromLeftOrRightEdge();
      for (const auto* edge : in_edges) {
        const auto* candidate_node = edge->FromNode();
        if (candidate_node == to_node) {
          continue;
        }
        if (candidate_node->GetInEdgeFrom(from_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != to_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

bool AdjustLaneChange(std::vector<const TopoNode*>* const result_node_vec) {
  if (result_node_vec->size() < 3) {
    return true;
  }
  if (!AdjustLaneChangeBackward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  if (!AdjustLaneChangeForward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  return true;
}

bool Reconstruct(
    const std::unordered_map<const TopoNode*, const TopoNode*>& came_from,
    const TopoNode* dest_node, std::vector<NodeWithRange>* result_nodes) {
  std::vector<const TopoNode*> result_node_vec;
  result_node_vec.push_back(dest_node);

  auto iter = came_from.find(dest_node);
  while (iter != came_from.end()) {
    result_node_vec.push_back(iter->second);
    iter = came_from.find(iter->second);
  }
  std::reverse(result_node_vec.begin(), result_node_vec.end());
  if (!AdjustLaneChange(&result_node_vec)) {
    AERROR << "Failed to adjust lane change";
    return false;
  }
  result_nodes->clear();
  for (const auto* node : result_node_vec) {
    result_nodes->emplace_back(node->OriginNode(), node->StartS(),
                               node->EndS());
  }
  return true;
}

}  // namespace

AStarStrategy::AStarStrategy(bool enable_change)
    : change_lane_enabled_(enable_change) {}

void AStarStrategy::Clear() {
  closed_set_.clear();
  open_set_.clear();
  came_from_.clear();
  enter_s_.clear();
  g_score_.clear();
}

double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                    const TopoNode* dest_node) {
  const auto& src_point = src_node->AnchorPoint();
  const auto& dest_point = dest_node->AnchorPoint();
  double distance = std::fabs(src_point.x() - dest_point.x()) +
                    std::fabs(src_point.y() - dest_point.y());
  return distance;
}

bool AStarStrategy::Search(const TopoGraph* graph,
                           const SubTopoGraph* sub_graph,
                           const TopoNode* src_node, const TopoNode* dest_node,
                           std::vector<NodeWithRange>* const result_nodes) {
  Clear();
  AINFO << "Start A* search algorithm.";

  std::priority_queue<SearchNode> open_set_detail;

  SearchNode src_search_node(src_node);
  src_search_node.f = HeuristicCost(src_node, dest_node);
  open_set_detail.push(src_search_node);

  open_set_.insert(src_node);
  g_score_[src_node] = 0.0;
  enter_s_[src_node] = src_node->StartS();

  SearchNode current_node;
  std::unordered_set<const TopoEdge*> next_edge_set;
  std::unordered_set<const TopoEdge*> sub_edge_set;
  while (!open_set_detail.empty()) {
    current_node = open_set_detail.top();
    const auto* from_node = current_node.topo_node;
    if (current_node.topo_node == dest_node) {
      if (!Reconstruct(came_from_, from_node, result_nodes)) {
        AERROR << "Failed to reconstruct route.";
        return false;
      }
      return true;
    }
    open_set_.erase(from_node);
    open_set_detail.pop();

    if (closed_set_.count(from_node) != 0) {
      // if showed before, just skip...
      continue;
    }
    closed_set_.emplace(from_node);

    // if residual_s is less than FLAGS_min_length_for_lane_change, only move
    // forward
    const auto& neighbor_edges =
        (GetResidualS(from_node) > FLAGS_min_length_for_lane_change &&
         change_lane_enabled_)
            ? from_node->OutToAllEdge()
            : from_node->OutToSucEdge();
    double tentative_g_score = 0.0;
    next_edge_set.clear();
    for (const auto* edge : neighbor_edges) {
      sub_edge_set.clear();
      sub_graph->GetSubInEdgesIntoSubGraph(edge, &sub_edge_set);
      next_edge_set.insert(sub_edge_set.begin(), sub_edge_set.end());
    }

    for (const auto* edge : next_edge_set) {
      const auto* to_node = edge->ToNode();
      if (closed_set_.count(to_node) == 1) {
        continue;
      }
      if (GetResidualS(edge, to_node) < FLAGS_min_length_for_lane_change) {
        continue;
      }
      tentative_g_score =
          g_score_[current_node.topo_node] + GetCostToNeighbor(edge);
      if (edge->Type() != TopoEdgeType::TET_FORWARD) {
        tentative_g_score -=
            (edge->FromNode()->Cost() + edge->ToNode()->Cost()) / 2;
      }
      double f = tentative_g_score + HeuristicCost(to_node, dest_node);
      if (open_set_.count(to_node) != 0 && f >= g_score_[to_node]) {
        continue;
      }
      // if to_node is reached by forward, reset enter_s to start_s
      if (edge->Type() == TopoEdgeType::TET_FORWARD) {
        enter_s_[to_node] = to_node->StartS();
      } else {
        // else, add enter_s with FLAGS_min_length_for_lane_change
        double to_node_enter_s =
            (enter_s_[from_node] + FLAGS_min_length_for_lane_change) /
            from_node->Length() * to_node->Length();
        // enter s could be larger than end_s but should be less than length
        to_node_enter_s = std::min(to_node_enter_s, to_node->Length());
        // if enter_s is larger than end_s and to_node is dest_node
        if (to_node_enter_s > to_node->EndS() && to_node == dest_node) {
          continue;
        }
        enter_s_[to_node] = to_node_enter_s;
      }

      g_score_[to_node] = f;
      SearchNode next_node(to_node);
      next_node.f = f;
      open_set_detail.push(next_node);
      came_from_[to_node] = from_node;
      if (open_set_.count(to_node) == 0) {
        open_set_.insert(to_node);
      }
    }
  }
  AERROR << "Failed to find goal lane with id: " << dest_node->LaneId();
  return false;
}

double AStarStrategy::GetResidualS(const TopoNode* node) {
  double start_s = node->StartS();
  const auto iter = enter_s_.find(node);
  if (iter != enter_s_.end()) {
    if (iter->second > node->EndS()) {
      return 0.0;
    }
    start_s = iter->second;
  } else {
    AWARN << "lane " << node->LaneId() << "(" << node->StartS() << ", "
          << node->EndS() << "not found in enter_s map";
  }
  double end_s = node->EndS();
  const TopoNode* succ_node = nullptr;
  for (const auto* edge : node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

double AStarStrategy::GetResidualS(const TopoEdge* edge,
                                   const TopoNode* to_node) {
  if (edge->Type() == TopoEdgeType::TET_FORWARD) {
    return std::numeric_limits<double>::max();
  }
  double start_s = to_node->StartS();
  const auto* from_node = edge->FromNode();
  const auto iter = enter_s_.find(from_node);
  if (iter != enter_s_.end()) {
    double temp_s = iter->second / from_node->Length() * to_node->Length();
    start_s = std::max(start_s, temp_s);
  } else {
    AWARN << "lane " << from_node->LaneId() << "(" << from_node->StartS()
          << ", " << from_node->EndS() << "not found in enter_s map";
  }
  double end_s = to_node->EndS();
  const TopoNode* succ_node = nullptr;
  for (const auto* edge : to_node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == to_node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

}  // namespace routing
}  // namespace apollo
