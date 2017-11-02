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
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/node_with_range.h"
#include "modules/routing/graph/range_utils.h"

namespace apollo {
namespace routing {

using apollo::common::adapter::AdapterManager;

namespace {

bool IsCloseEnough(double value_1, double value_2) {
  constexpr double kEpsilon = 1e-6;
  return std::fabs(value_1 - value_2) < kEpsilon;
}

std::string GetRoadId(
    const std::vector<std::vector<NodeWithRange>>& node_vecs) {
  std::unordered_set<std::string> road_id_set;
  std::string road_id = "";
  for (const auto& node_vec : node_vecs) {
    for (const auto& node : node_vec) {
      road_id_set.emplace(node.RoadId());
    }
  }
  for (const auto& id : road_id_set) {
    if (road_id.empty()) {
      road_id = id;
    } else {
      road_id += "-" + id;
    }
  }
  return road_id;
}

const NodeWithRange& GetLargestRange(
    const std::vector<NodeWithRange>& node_vec) {
  CHECK(!node_vec.empty());
  size_t result_idx = 0;
  double result_range_length = 0.0;
  for (size_t i = 1; i < node_vec.size(); ++i) {
    if (node_vec[i].Length() > result_range_length) {
      result_range_length = node_vec[i].Length();
      result_idx = i;
    }
  }
  return node_vec[result_idx];
}

void GetNodesOfWaysBasedOnVirtual(
    const std::vector<NodeWithRange>& nodes,
    std::vector<std::vector<NodeWithRange>>* const nodes_of_ways) {
  ADEBUG << "Cut routing ways based on is_virtual.";
  CHECK_NOTNULL(nodes_of_ways);
  nodes_of_ways->clear();
  auto iter = nodes.begin();
  while (iter != nodes.end()) {
    auto next = iter + 1;
    while (next != nodes.end() && next->IsVirtual() == iter->IsVirtual()) {
      ++next;
    }
    nodes_of_ways->emplace_back(iter, next);
    iter = next;
  }
}

void GetNodesOfWays(
    const std::vector<NodeWithRange>& nodes,
    std::vector<std::vector<NodeWithRange>>* const nodes_of_ways) {
  ADEBUG << "Cut routing ways based on road id.";
  CHECK_NOTNULL(nodes_of_ways);
  nodes_of_ways->clear();
  auto iter = nodes.begin();
  while (iter != nodes.end()) {
    auto next = iter + 1;
    while (next != nodes.end() && next->RoadId() == iter->RoadId()) {
      ++next;
    }
    nodes_of_ways->emplace_back(iter, next);
    iter = next;
  }
}

bool ExtractBasicPassages(
    const std::vector<NodeWithRange>& nodes,
    std::vector<std::vector<NodeWithRange>>* const nodes_of_passages,
    std::vector<ChangeLaneType>* change_lane_type) {
  CHECK(!nodes.empty());
  nodes_of_passages->clear();
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
      if (edge->Type() == TET_LEFT) {
        change_lane_type->emplace_back(LEFT);
      } else {
        change_lane_type->emplace_back(RIGHT);
      }
      nodes_of_passages->push_back(nodes_of_passage);
      nodes_of_passage.clear();
    }
    nodes_of_passage.push_back(nodes.at(i));
  }
  change_lane_type->emplace_back(FORWARD);
  nodes_of_passages->push_back(nodes_of_passage);
  return true;
}

bool IsReachableFromWithChangeLane(const TopoNode* node,
                                   const std::vector<NodeWithRange>& to_nodes,
                                   NodeWithRange* reachable_node) {
  for (const auto& to_node : to_nodes) {
    auto edge = to_node.GetTopoNode()->GetInEdgeFrom(node);
    if (edge != nullptr) {
      if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
        *reachable_node = to_node;
        return true;
      }
    }
  }
  return false;
}

bool IsReachableToWithChangeLane(const TopoNode* node,
                                 const std::vector<NodeWithRange>& from_nodes,
                                 NodeWithRange* reachable_node) {
  for (const auto& from_node : from_nodes) {
    auto edge = from_node.GetTopoNode()->GetOutEdgeTo(node);
    if (edge != nullptr) {
      if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
        *reachable_node = from_node;
        return true;
      }
    }
  }
  return false;
}

void ExtendBackward(bool enable_use_road_id,
                    const std::vector<NodeWithRange>& nodes_of_prev_passage,
                    const TopoRangeManager& range_manager,
                    std::vector<NodeWithRange>* const nodes_of_curr_passage) {
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : *nodes_of_curr_passage) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }
  auto& front_node = nodes_of_curr_passage->front();
  // if front node starts at middle
  if (!IsCloseEnough(front_node.StartS(), 0.0)) {
    if (!range_manager.Find(front_node.GetTopoNode())) {
      if (IsCloseEnough(nodes_of_prev_passage.front().StartS(), 0.0)) {
        front_node.SetStartS(0.0);
      } else {
        double temp_s = nodes_of_prev_passage.front().StartS() /
                        nodes_of_prev_passage.front().FullLength() *
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
         nodes_of_curr_passage->front().GetTopoNode()->InFromPreEdge()) {
      const auto& pred_node = edge->FromNode();
      // if pred node is not in the same road
      // if (enable_use_road_id && pred_node->RoadId() !=
      // nodes_of_curr_passage->front().RoadId()) {
      //   continue;
      // }

      // if pred node has been inserted
      if (node_set_of_curr_passage.find(pred_node) !=
          node_set_of_curr_passage.end()) {
        continue;
      }
      // if pred node is reachable from prev passage
      NodeWithRange reachable_node(pred_node, 0, 1);
      if (IsReachableToWithChangeLane(pred_node, nodes_of_prev_passage,
                                      &reachable_node)) {
        if (range_manager.Find(pred_node)) {
          double black_s_end = range_manager.RangeEnd(pred_node);
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
      nodes_of_curr_passage->insert(nodes_of_curr_passage->begin(),
                                    node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

void ExtendForward(bool enable_use_road_id,
                   const std::vector<NodeWithRange>& nodes_of_next_passage,
                   const TopoRangeManager& range_manager,
                   std::vector<NodeWithRange>* const nodes_of_curr_passage) {
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : *nodes_of_curr_passage) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }
  auto& back_node = nodes_of_curr_passage->back();
  if (!IsCloseEnough(back_node.EndS(), back_node.FullLength())) {
    if (!range_manager.Find(back_node.GetTopoNode())) {
      if (IsCloseEnough(nodes_of_next_passage.back().EndS(),
                        nodes_of_next_passage.back().FullLength())) {
        back_node.SetEndS(back_node.FullLength());
      } else {
        double temp_s = nodes_of_next_passage.back().EndS() /
                        nodes_of_next_passage.back().FullLength() *
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
         nodes_of_curr_passage->back().GetTopoNode()->OutToSucEdge()) {
      const auto& succ_node = edge->ToNode();
      // if succ node is not in the same road
      // if (enable_use_road_id &&
      //     succ_node->RoadId() != nodes_of_curr_passage->back().RoadId()) {
      //   continue;
      // }
      // if succ node has been inserted
      if (node_set_of_curr_passage.find(succ_node) !=
          node_set_of_curr_passage.end()) {
        continue;
      }
      // if next passage is reachable from succ node
      NodeWithRange reachable_node(succ_node, 0, 1.0);
      if (IsReachableFromWithChangeLane(succ_node, nodes_of_next_passage,
                                        &reachable_node)) {
        if (range_manager.Find(succ_node)) {
          double black_s_start = range_manager.RangeStart(succ_node);
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
      nodes_of_curr_passage->push_back(node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

void ExtendPassages(
    bool enable_use_road_id, const TopoRangeManager& range_manager,
    std::vector<std::vector<NodeWithRange>>* const nodes_of_passages) {
  int passage_num = nodes_of_passages->size();
  for (int i = 0; i < passage_num; ++i) {
    if (i < passage_num - 1) {
      ExtendForward(enable_use_road_id, nodes_of_passages->at(i + 1),
                    range_manager, &(nodes_of_passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(enable_use_road_id, nodes_of_passages->at(i - 1),
                     range_manager, &(nodes_of_passages->at(i)));
    }
  }
  for (int i = passage_num - 1; i >= 0; --i) {
    if (i < passage_num - 1) {
      ExtendForward(enable_use_road_id, nodes_of_passages->at(i + 1),
                    range_manager, &(nodes_of_passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(enable_use_road_id, nodes_of_passages->at(i - 1),
                     range_manager, &(nodes_of_passages->at(i)));
    }
  }
}

void LaneNodesToPassageRegion(const std::vector<NodeWithRange>& nodes,
                              Passage* const passage) {
  for (const auto& node : nodes) {
    LaneSegment* seg = passage->add_segment();
    seg->set_id(node.LaneId());
    seg->set_start_s(node.StartS());
    seg->set_end_s(node.EndS());
  }
}

double CalculateDistance(const std::vector<NodeWithRange>& nodes) {
  double distance = nodes.at(0).EndS() - nodes.at(0).StartS();
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge->Type() != TET_FORWARD) {
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

}  // namespace

bool ResultGenerator::GeneratePassageRegion(
    const std::string& map_version, const RoutingRequest& request,
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager, RoutingResponse* const result) {
  AdapterManager::FillRoutingResponseHeader(FLAGS_node_name, result);

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
  std::vector<std::vector<NodeWithRange>> nodes_of_ways;
  if (FLAGS_use_road_id) {
    GetNodesOfWays(nodes, &nodes_of_ways);
  } else {
    GetNodesOfWaysBasedOnVirtual(nodes, &nodes_of_ways);
  }
  std::vector<std::string> road_id_of_ways;
  for (const auto& way : nodes_of_ways) {
    std::vector<std::vector<NodeWithRange>> nodes_of_passages;
    if (way.empty()) {
      return false;
    }
    std::vector<ChangeLaneType> change_lane_type;
    if (!ExtractBasicPassages(way, &nodes_of_passages, &change_lane_type)) {
      return false;
    }

    ExtendPassages(FLAGS_use_road_id, range_manager, &nodes_of_passages);

    RoadSegment road_segment;
    const auto road_id = GetRoadId(nodes_of_passages);
    road_segment.set_id(road_id);
    int index = -1;
    for (const auto& nodes_of_passage : nodes_of_passages) {
      auto* passage = road_segment.add_passage();
      LaneNodesToPassageRegion(nodes_of_passage, passage);
      passage->set_change_lane_type(change_lane_type[++index]);
    }
    road_segment.mutable_passage()->rbegin()->set_can_exit(true);
    result->add_road()->CopyFrom(road_segment);
    PrintDebugInfo(road_id, nodes_of_ways);
  }

  return true;
}

}  // namespace routing
}  // namespace apollo
