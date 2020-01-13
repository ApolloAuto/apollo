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
#include "modules/routing/core/result_generator.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"

namespace apollo {
namespace routing {

using apollo::common::util::ContainsKey;

bool IsCloseEnough(double value_1, double value_2) {
  static constexpr double kEpsilon = 1e-6;
  return std::fabs(value_1 - value_2) < kEpsilon;
}

const NodeWithRange& GetLargestRange(
    const std::vector<NodeWithRange>& node_vec) {
  CHECK(!node_vec.empty());
  size_t result_idx = 0;
  double result_range_length = 0.0;
  for (size_t i = 0; i < node_vec.size(); ++i) {
    if (node_vec[i].Length() > result_range_length) {
      result_range_length = node_vec[i].Length();
      result_idx = i;
    }
  }
  return node_vec[result_idx];
}

bool ResultGenerator::ExtractBasicPassages(
    const std::vector<NodeWithRange>& nodes,
    std::vector<PassageInfo>* const passages) {
  CHECK(!nodes.empty());
  passages->clear();
  std::vector<NodeWithRange> nodes_of_passage;
  nodes_of_passage.push_back(nodes.at(0));
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge == nullptr) {
      AERROR << "Get null pointer to edge from " << nodes.at(i - 1).LaneId()
             << " to " << nodes.at(i).LaneId();
      return false;
    }
    if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
      auto change_lane_type = LEFT;
      if (edge->Type() == TET_RIGHT) {
        change_lane_type = RIGHT;
      }
      passages->emplace_back(nodes_of_passage, change_lane_type);
      nodes_of_passage.clear();
    }
    nodes_of_passage.push_back(nodes.at(i));
  }
  passages->emplace_back(nodes_of_passage, FORWARD);
  return true;
}

bool ResultGenerator::IsReachableFromWithChangeLane(
    const TopoNode* from_node, const PassageInfo& to_nodes,
    NodeWithRange* reachable_node) {
  for (const auto& to_node : to_nodes.nodes) {
    auto edge = to_node.GetTopoNode()->GetInEdgeFrom(from_node);
    if (edge != nullptr &&
        (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT)) {
      *reachable_node = to_node;
      return true;
    }
  }
  return false;
}

bool ResultGenerator::IsReachableToWithChangeLane(
    const TopoNode* to_node, const PassageInfo& from_nodes,
    NodeWithRange* reachable_node) {
  for (const auto& from_node : from_nodes.nodes) {
    auto edge = from_node.GetTopoNode()->GetOutEdgeTo(to_node);
    if (edge != nullptr &&
        (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT)) {
      *reachable_node = from_node;
      return true;
    }
  }
  return false;
}

void ResultGenerator::ExtendBackward(const TopoRangeManager& range_manager,
                                     const PassageInfo& prev_passage,
                                     PassageInfo* const curr_passage) {
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : curr_passage->nodes) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }
  auto& front_node = curr_passage->nodes.front();
  // if front node starts at middle
  if (!IsCloseEnough(front_node.StartS(), 0.0)) {
    if (!range_manager.Find(front_node.GetTopoNode())) {
      if (IsCloseEnough(prev_passage.nodes.front().StartS(), 0.0)) {
        front_node.SetStartS(0.0);
      } else {
        double temp_s = prev_passage.nodes.front().StartS() /
                        prev_passage.nodes.front().FullLength() *
                        front_node.FullLength();
        front_node.SetStartS(temp_s);
      }
    } else {
      return;
    }
  }

  bool allowed_to_explore = true;
  while (allowed_to_explore) {
    std::vector<NodeWithRange> pred_set;
    for (const auto& edge :
         curr_passage->nodes.front().GetTopoNode()->InFromPreEdge()) {
      const auto& pred_node = edge->FromNode();

      // if pred node has been inserted
      if (ContainsKey(node_set_of_curr_passage, pred_node)) {
        continue;
      }
      // if pred node is reachable from prev passage
      NodeWithRange reachable_node(pred_node, 0, 1);
      if (IsReachableToWithChangeLane(pred_node, prev_passage,
                                      &reachable_node)) {
        const auto* pred_range = range_manager.Find(pred_node);
        if (pred_range != nullptr && !pred_range->empty()) {
          double black_s_end = pred_range->back().EndS();
          if (!IsCloseEnough(black_s_end, pred_node->Length())) {
            pred_set.emplace_back(pred_node, black_s_end, pred_node->Length());
          }
        } else {
          pred_set.emplace_back(pred_node, 0.0, pred_node->Length());
        }
      }
    }
    if (pred_set.empty()) {
      allowed_to_explore = false;
    } else {
      allowed_to_explore = true;
      const auto& node_to_insert = GetLargestRange(pred_set);
      curr_passage->nodes.insert(curr_passage->nodes.begin(), node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

void ResultGenerator::ExtendForward(const TopoRangeManager& range_manager,
                                    const PassageInfo& next_passage,
                                    PassageInfo* const curr_passage) {
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : curr_passage->nodes) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }
  auto& back_node = curr_passage->nodes.back();
  if (!IsCloseEnough(back_node.EndS(), back_node.FullLength())) {
    if (!range_manager.Find(back_node.GetTopoNode())) {
      if (IsCloseEnough(next_passage.nodes.back().EndS(),
                        next_passage.nodes.back().FullLength())) {
        back_node.SetEndS(back_node.FullLength());
      } else {
        double temp_s = next_passage.nodes.back().EndS() /
                        next_passage.nodes.back().FullLength() *
                        back_node.FullLength();
        back_node.SetEndS(std::min(temp_s, back_node.FullLength()));
      }
    } else {
      return;
    }
  }

  bool allowed_to_explore = true;
  while (allowed_to_explore) {
    std::vector<NodeWithRange> succ_set;
    for (const auto& edge :
         curr_passage->nodes.back().GetTopoNode()->OutToSucEdge()) {
      const auto& succ_node = edge->ToNode();
      // if succ node has been inserted
      if (ContainsKey(node_set_of_curr_passage, succ_node)) {
        continue;
      }
      // if next passage is reachable from succ node
      NodeWithRange reachable_node(succ_node, 0, 1.0);
      if (IsReachableFromWithChangeLane(succ_node, next_passage,
                                        &reachable_node)) {
        const auto* succ_range = range_manager.Find(succ_node);
        if (succ_range != nullptr && !succ_range->empty()) {
          double black_s_start = succ_range->front().StartS();
          if (!IsCloseEnough(black_s_start, 0.0)) {
            succ_set.emplace_back(succ_node, 0.0, black_s_start);
          }
        } else {
          if (IsCloseEnough(reachable_node.EndS(),
                            reachable_node.FullLength())) {
            succ_set.emplace_back(succ_node, 0.0, succ_node->Length());
          } else {
            double push_end_s = reachable_node.EndS() /
                                reachable_node.FullLength() *
                                succ_node->Length();
            succ_set.emplace_back(succ_node, 0.0, push_end_s);
          }
        }
      }
    }
    if (succ_set.empty()) {
      allowed_to_explore = false;
    } else {
      allowed_to_explore = true;
      const auto& node_to_insert = GetLargestRange(succ_set);
      curr_passage->nodes.push_back(node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

void ResultGenerator::ExtendPassages(const TopoRangeManager& range_manager,
                                     std::vector<PassageInfo>* const passages) {
  int passage_num = static_cast<int>(passages->size());
  for (int i = 0; i < passage_num; ++i) {
    if (i < passage_num - 1) {
      ExtendForward(range_manager, passages->at(i + 1), &(passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(range_manager, passages->at(i - 1), &(passages->at(i)));
    }
  }
  for (int i = passage_num - 1; i >= 0; --i) {
    if (i < passage_num - 1) {
      ExtendForward(range_manager, passages->at(i + 1), &(passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(range_manager, passages->at(i - 1), &(passages->at(i)));
    }
  }
}

void LaneNodesToPassageRegion(
    const std::vector<NodeWithRange>::const_iterator first,
    const std::vector<NodeWithRange>::const_iterator last,
    Passage* const passage) {
  for (auto it = first; it != last; ++it) {
    LaneSegment* seg = passage->add_segment();
    seg->set_id(it->LaneId());
    seg->set_start_s(it->StartS());
    seg->set_end_s(it->EndS());
  }
}

void LaneNodesToPassageRegion(const std::vector<NodeWithRange>& nodes,
                              Passage* const passage) {
  return LaneNodesToPassageRegion(nodes.begin(), nodes.end(), passage);
}

double CalculateDistance(const std::vector<NodeWithRange>& nodes) {
  double distance = nodes.at(0).EndS() - nodes.at(0).StartS();
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge == nullptr || edge->Type() != TET_FORWARD) {
      continue;
    }
    distance += nodes.at(i).EndS() - nodes.at(i).StartS();
  }
  return distance;
}

void PrintDebugInfo(const std::string& road_id,
                    const std::vector<std::vector<NodeWithRange>>& nodes) {
  AINFO << "road id: " << road_id;
  for (size_t j = 0; j < nodes.size(); ++j) {
    AINFO << "\tPassage " << j;
    for (const auto& node : nodes[j]) {
      AINFO << "\t\t" << node.LaneId() << "   (" << node.StartS() << ", "
            << node.EndS() << ")";
    }
  }
}

bool ResultGenerator::GeneratePassageRegion(
    const std::string& map_version, const RoutingRequest& request,
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager, RoutingResponse* const result) {
  if (!GeneratePassageRegion(nodes, range_manager, result)) {
    return false;
  }

  result->set_map_version(map_version);
  result->mutable_measurement()->set_distance(CalculateDistance(nodes));
  result->mutable_routing_request()->CopyFrom(request);
  return true;
}

bool ResultGenerator::GeneratePassageRegion(
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager, RoutingResponse* const result) {
  std::vector<PassageInfo> passages;
  if (!ExtractBasicPassages(nodes, &passages)) {
    return false;
  }
  ExtendPassages(range_manager, &passages);

  CreateRoadSegments(passages, result);

  return true;
}

void ResultGenerator::AddRoadSegment(
    const std::vector<PassageInfo>& passages,
    const std::pair<std::size_t, std::size_t>& start,
    const std::pair<std::size_t, std::size_t>& end, RoutingResponse* result) {
  auto* road = result->add_road();
  road->set_id(passages[start.first].nodes[start.second].RoadId());
  for (std::size_t i = start.first; i <= end.first && i < passages.size();
       ++i) {
    auto* passage = road->add_passage();
    auto start_iter =
        passages[i].nodes.cbegin() + (i == start.first ? start.second : 0);
    auto end_iter = passages[i].nodes.cbegin() +
                    (i == end.first ? end.second : passages[i].nodes.size());
    LaneNodesToPassageRegion(start_iter, end_iter, passage);
    if (start.first == end.first) {
      passage->set_change_lane_type(FORWARD);
      passage->set_can_exit(true);
    } else {
      passage->set_change_lane_type(passages[i].change_lane_type);
      passage->set_can_exit(i == end.first);
    }
  }
}

void ResultGenerator::CreateRoadSegments(
    const std::vector<PassageInfo>& passages, RoutingResponse* result) {
  CHECK(!passages.empty()) << "passages empty";
  NodeWithRange fake_node_range(passages.front().nodes.front());
  bool in_change_lane = false;
  std::pair<std::size_t, std::size_t> start_index(0, 0);
  for (std::size_t i = 0; i < passages.size(); ++i) {
    const auto& curr_nodes = passages[i].nodes;
    for (std::size_t j = 0; j < curr_nodes.size(); ++j) {
      if ((i + 1 < passages.size() &&
           IsReachableToWithChangeLane(curr_nodes[j].GetTopoNode(),
                                       passages[i + 1], &fake_node_range)) ||
          (i > 0 &&
           IsReachableFromWithChangeLane(curr_nodes[j].GetTopoNode(),
                                         passages[i - 1], &fake_node_range))) {
        if (!in_change_lane) {
          start_index = {i, j};
          in_change_lane = true;
        }
      } else {
        if (in_change_lane) {
          AddRoadSegment(passages, start_index, {i, j}, result);
        }
        AddRoadSegment(passages, {i, j}, {i, j + 1}, result);
        in_change_lane = false;
      }
    }
  }
  if (in_change_lane) {
    AddRoadSegment(passages, start_index,
                   {passages.size() - 1, passages.back().nodes.size()}, result);
  }
}

}  // namespace routing
}  // namespace apollo
