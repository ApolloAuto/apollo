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
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/routing/core/result_generator.h"

#include "modules/common/log.h"

#include "modules/common/time/time.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/range_utils.h"
#include "modules/routing/graph/node_with_range.h"

namespace apollo {
namespace routing {

namespace {

bool IsCloseEnough(double value_1, double value_2) {
  if (fabs(value_1 - value_2) < 1e-6) {
    return true;
  }
  return false;
}

std::string GetRoadId(
    const std::vector<std::vector<NodeWithRange> >& node_vecs) {
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

void NodeVecToSet(const std::vector<NodeWithRange>& vec,
                  std::unordered_set<const TopoNode*>* const set) {
  for (const auto& node : vec) {
    set->insert(node.GetTopoNode());
  }
}

NodeWithRange BuildNodeRange(const TopoNode* node,
                             double start_s,
                             double end_s) {
  NodeSRange range(start_s, end_s);
  NodeWithRange node_with_range(range, node);
  return node_with_range;
}

const NodeWithRange& GetLargestRange(
    const std::vector<NodeWithRange>& node_vec) {
  assert(!node_vec.empty());
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
    std::vector<std::vector<NodeWithRange> >* const nodes_of_ways) {
  AINFO << "Cut routing ways based on is_virtual.";
  assert(!nodes.empty());
  nodes_of_ways->clear();
  std::vector<NodeWithRange> nodes_of_way;
  std::unordered_set<std::string> road_ids_of_way;
  nodes_of_way.push_back(nodes.at(0));
  road_ids_of_way.emplace(nodes.at(0).RoadId());
  bool last_is_virtual = nodes.at(0).IsVirtual();
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    if ((*iter).IsVirtual() != last_is_virtual) {
      nodes_of_ways->push_back(nodes_of_way);
      nodes_of_way.clear();
      road_ids_of_way.clear();
      last_is_virtual = (*iter).IsVirtual();
    }
    nodes_of_way.push_back(*iter);
    road_ids_of_way.emplace((*iter).RoadId());
  }
  nodes_of_ways->push_back(nodes_of_way);
}

void GetNodesOfWays(
    const std::vector<NodeWithRange>& nodes,
    std::vector<std::vector<NodeWithRange> >* const nodes_of_ways) {
  AINFO << "Cut routing ways based on road id.";
  assert(!nodes.empty());
  nodes_of_ways->clear();
  std::vector<NodeWithRange> nodes_of_way;
  nodes_of_way.push_back(nodes.at(0));
  std::string last_road_id = nodes.at(0).RoadId();
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    if ((*iter).RoadId() != last_road_id) {
      nodes_of_ways->push_back(nodes_of_way);
      nodes_of_way.clear();
      last_road_id = (*iter).RoadId();
    }
    nodes_of_way.push_back(*iter);
  }
  nodes_of_ways->push_back(nodes_of_way);
}

bool ExtractBasicPassages(
    const std::vector<NodeWithRange>& nodes,
    std::vector<std::vector<NodeWithRange> >* const nodes_of_passages,
    std::vector<RoutingResponse::LaneChangeInfo::Type>* const
    lane_change_types) {
  assert(!nodes.empty());
  nodes_of_passages->clear();
  lane_change_types->clear();
  std::vector<NodeWithRange> nodes_of_passage;
  nodes_of_passage.push_back(nodes.at(0));
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge = nodes.at(i - 1).GetTopoNode()
        ->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge == nullptr) {
      AERROR << "Get null pointer to edge from "
          << nodes.at(i - 1).LaneId() << " to " << nodes.at(i).LaneId();
      return false;
    }
    if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
      nodes_of_passages->push_back(nodes_of_passage);
      nodes_of_passage.clear();
      if (edge->Type() == TET_LEFT) {
        lane_change_types->push_back(
            RoutingResponse::LaneChangeInfo::LEFT_FORWARD);
      } else {
        lane_change_types->push_back(
            RoutingResponse::LaneChangeInfo::RIGHT_FORWARD);
      }
    }
    nodes_of_passage.push_back(nodes.at(i));
  }
  nodes_of_passages->push_back(nodes_of_passage);
  return true;
}

bool IsReachableFrom(const TopoNode* node,
                     const std::vector<NodeWithRange>& to_nodes,
                     const NodeWithRange** const reachable_node) {
  for (const auto& to_node : to_nodes) {
    auto edge = to_node.GetTopoNode()->GetInEdgeFrom(node);
    if (edge != nullptr) {
      if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
        *reachable_node = &to_node;
        return true;
      }
    }
  }
  return false;
}

bool IsReachableTo(const TopoNode* node,
                   const std::vector<NodeWithRange>& from_nodes,
                   const NodeWithRange** const reachable_node) {
  for (const auto& from_node : from_nodes) {
    auto edge = from_node.GetTopoNode()->GetOutEdgeTo(node);
    if (edge != nullptr) {
      if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
        *reachable_node = &from_node;
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
  NodeVecToSet(*nodes_of_curr_passage, &node_set_of_curr_passage);
  auto& front_node = nodes_of_curr_passage->front();
  // if front node starts at middle
  if (!IsCloseEnough(front_node.StartS(), 0.0)) {
    if (!range_manager.Find(front_node.GetTopoNode())) {
      if (IsCloseEnough(nodes_of_prev_passage.front().StartS(), 0.0)) {
        front_node.SetStartS(0.0);
      } else {
        double temp_s = nodes_of_prev_passage.front().StartS()
            / nodes_of_prev_passage.front().FullLength()
            * front_node.FullLength();
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
      if (pred_node->RoadId() != nodes_of_curr_passage->front().RoadId()) {
        continue;
      }
      // if pred node has been inserted
      if (enable_use_road_id &&
            node_set_of_curr_passage.find(pred_node)
                != node_set_of_curr_passage.end()) {
        continue;
      }
      // if pred node is reachable from prev passage
      const NodeWithRange* reachable_node = nullptr;
      if (IsReachableTo(pred_node, nodes_of_prev_passage, &reachable_node)) {
        if (range_manager.Find(pred_node)) {
          double black_s_end = range_manager.RangeEnd(pred_node);
          if (!IsCloseEnough(black_s_end, pred_node->Length())) {
            pred_set.push_back(BuildNodeRange(pred_node,
                                              black_s_end,
                                              pred_node->Length()));
          }
        } else {
          pred_set.push_back(BuildNodeRange(pred_node,
                                            0.0,
                                            pred_node->Length()));
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
  NodeVecToSet(*nodes_of_curr_passage, &node_set_of_curr_passage);
  auto& back_node = nodes_of_curr_passage->back();
  if (!IsCloseEnough(back_node.EndS(), back_node.FullLength())) {
    if (!range_manager.Find(back_node.GetTopoNode())) {
      if (IsCloseEnough(nodes_of_next_passage.back().EndS(),
                        nodes_of_next_passage.back().FullLength())) {
        back_node.SetEndS(back_node.FullLength());
      } else {
        double temp_s = nodes_of_next_passage.back().EndS() /
            nodes_of_next_passage.back().FullLength() * back_node.FullLength();
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
      if (enable_use_road_id &&
            succ_node->RoadId() != nodes_of_curr_passage->back().RoadId()) {
        continue;
      }
      // if succ node has been inserted
      if (node_set_of_curr_passage.find(succ_node)
            != node_set_of_curr_passage.end()) {
        continue;
      }
      // if next passage is reachable from succ node
      const NodeWithRange* reachable_node = nullptr;
      if (IsReachableFrom(succ_node, nodes_of_next_passage, &reachable_node)) {
        if (range_manager.Find(succ_node)) {
          double black_s_start = range_manager.RangeStart(succ_node);
          if (!IsCloseEnough(black_s_start, 0.0)) {
            succ_set.push_back(BuildNodeRange(succ_node, 0.0, black_s_start));
          }
        } else {
          if (IsCloseEnough(reachable_node->EndS(),
                            reachable_node->FullLength())) {
            succ_set.push_back(BuildNodeRange(succ_node,
                                              0.0,
                                              succ_node->Length()));
          } else {
            double push_end_s = reachable_node->EndS() /
                reachable_node->FullLength() * succ_node->Length();
            succ_set.push_back(BuildNodeRange(succ_node, 0.0, push_end_s));
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
    bool enable_use_road_id,
    const TopoRangeManager& range_manager,
    std::vector<std::vector<NodeWithRange> >* const nodes_of_passages) {
  int passage_num = nodes_of_passages->size();
  for (int i = 0; i < passage_num; ++i) {
    if (i < passage_num - 1) {
      ExtendForward(enable_use_road_id,
                    nodes_of_passages->at(i + 1),
                    range_manager,
                    &(nodes_of_passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(enable_use_road_id,
                     nodes_of_passages->at(i - 1),
                     range_manager,
                     &(nodes_of_passages->at(i)));
    }
  }
  for (int i = passage_num - 1; i >= 0; --i) {
    if (i < passage_num - 1) {
      ExtendForward(enable_use_road_id,
                    nodes_of_passages->at(i + 1),
                    range_manager,
                    &(nodes_of_passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(enable_use_road_id,
                     nodes_of_passages->at(i - 1),
                     range_manager,
                     &(nodes_of_passages->at(i)));
    }
  }
}

void LaneNodesToPassageRegion(const std::vector<NodeWithRange>& nodes,
                              RoutingResponse::PassageRegion* const region) {
  for (const auto& node : nodes) {
    RoutingResponse::LaneSegment* seg = region->add_segment();
    seg->set_id(node.LaneId());
    seg->set_start_s(node.StartS());
    seg->set_end_s(node.EndS());
  }
}

double CalculateDistance(const std::vector<NodeWithRange>& nodes) {
  double distance = nodes.at(0).EndS() - nodes.at(0).StartS();
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge = nodes.at(i - 1).GetTopoNode()
            ->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge->Type() != TET_FORWARD) {
      continue;
    }
    distance += nodes.at(i).EndS() - nodes.at(i).StartS();
  }
  return distance;
}

void PrintDebugInfo(
    const std::vector<std::vector<std::vector<NodeWithRange> > >&
        nodes_of_passages_of_ways,
    const std::vector<std::string>& road_id_of_ways,
    const std::vector<bool>& is_virtual_of_ways) {
  for (size_t i = 0; i < nodes_of_passages_of_ways.size(); ++i) {
    if (is_virtual_of_ways[i]) {
      AINFO << "----------Way " << i << " juntion----------";
    } else {
      AINFO << "----------Way " << i << " road----------";
    }
    AINFO << "road id: " << road_id_of_ways[i].c_str();
    for (size_t j = 0; j < nodes_of_passages_of_ways[i].size(); ++j) {
      AINFO << "\tPassage " << j;
      for (const auto& node : nodes_of_passages_of_ways[i][j]) {
        AINFO << "\t\t" << node.LaneId() << "   (" << node.StartS() << ", "
            << node.EndS() << ")";
      }
    }
  }
}

}  // namespace

ResultGenerator::ResultGenerator() : _sequence_num(0) { }

bool ResultGenerator::GeneratePassageRegion(
    const std::string& map_version,
    const RoutingRequest& request,
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager,
    RoutingResponse* const result) {

  result->mutable_header()->set_timestamp_sec(
      ::apollo::common::time::Clock::NowInSecond());
  result->mutable_header()->set_module_name(FLAGS_node_name);
  result->mutable_header()->set_sequence_num(++_sequence_num);

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
    const TopoRangeManager& range_manager,
    RoutingResponse* const result) {
  std::vector<std::vector<NodeWithRange> > nodes_of_ways;
  if (FLAGS_use_road_id) {
    GetNodesOfWays(nodes, &nodes_of_ways);
  } else {
    GetNodesOfWaysBasedOnVirtual(nodes, &nodes_of_ways);
  }
  std::vector<std::vector<std::vector<NodeWithRange> > >
      nodes_of_passages_of_ways;
  std::vector<std::vector<RoutingResponse::LaneChangeInfo::Type> >
      lane_change_types_of_ways;
  std::vector<std::string> road_id_of_ways;
  std::vector<bool> is_virtual_of_ways;
  for (size_t i = 0; i < nodes_of_ways.size(); ++i) {
    std::vector<std::vector<NodeWithRange> > nodes_of_passages;
    std::vector<RoutingResponse::LaneChangeInfo::Type> lane_change_types;
    if (nodes_of_ways[i].empty()) {
      return false;
    }
    if (!nodes_of_ways[i][0].IsVirtual()) {
      is_virtual_of_ways.push_back(false);
      if (!ExtractBasicPassages(nodes_of_ways[i],
                                &nodes_of_passages,
                                &lane_change_types)) {
        return false;
      }

      ExtendPassages(FLAGS_use_road_id, range_manager, &nodes_of_passages);

      road_id_of_ways.push_back(GetRoadId(nodes_of_passages));
      nodes_of_passages_of_ways.push_back(std::move(nodes_of_passages));
      lane_change_types_of_ways.push_back(std::move(lane_change_types));
    } else {
      is_virtual_of_ways.push_back(true);
      nodes_of_passages.push_back(nodes_of_ways[i]);

      road_id_of_ways.push_back(GetRoadId(nodes_of_passages));
      nodes_of_passages_of_ways.push_back(std::move(nodes_of_passages));
      lane_change_types_of_ways.push_back(std::move(lane_change_types));
    }
  }

  PrintDebugInfo(nodes_of_passages_of_ways,
                 road_id_of_ways,
                 is_virtual_of_ways);

  for (size_t i = 0; i < nodes_of_passages_of_ways.size(); ++i) {
    const auto& nodes_of_passages = nodes_of_passages_of_ways[i];
    if (!is_virtual_of_ways[i]) {
      RoutingResponse::Road road;
      road.set_id(road_id_of_ways[i]);
      const auto& front_node = nodes_of_passages.front().front();
      road.mutable_in_lane()->set_id(front_node.LaneId());
      road.mutable_in_lane()->set_s(front_node.StartS());
      const auto& back_node = nodes_of_passages.back().back();
      road.mutable_out_lane()->set_id(back_node.LaneId());
      road.mutable_out_lane()->set_s(back_node.EndS());
      for (const auto& nodes_of_passage : nodes_of_passages) {
        LaneNodesToPassageRegion(nodes_of_passage, road.add_passage_region());
      }
      for (size_t j = 0; j < lane_change_types_of_ways[i].size(); ++j) {
        RoutingResponse::LaneChangeInfo* lc_info = road.add_lane_change_info();
        lc_info->set_type(lane_change_types_of_ways[i][j]);
        lc_info->set_start_passage_region_index(j);
        lc_info->set_end_passage_region_index(j + 1);
      }
      result->add_route()->mutable_road_info()->CopyFrom(road);
    } else {
      RoutingResponse::Junction junction;
      junction.set_id(road_id_of_ways[i]);
      if (i > 0) {
        junction.set_in_road_id(road_id_of_ways[i - 1]);
      } else {
        junction.set_in_road_id("UNKNOWN");
      }
      if (i < road_id_of_ways.size() - 1) {
        junction.set_out_road_id(road_id_of_ways[i + 1]);
      } else {
        junction.set_out_road_id("UNKNOWN");
      }
      RoutingResponse::PassageRegion region;
      for (const auto& node : nodes_of_passages.front()) {
        RoutingResponse::LaneSegment* seg = region.add_segment();
        seg->set_id(node.LaneId());
        seg->set_start_s(node.StartS());
        seg->set_end_s(node.EndS());
      }
      junction.mutable_passage_region()->CopyFrom(region);
      result->add_route()->mutable_junction_info()->CopyFrom(junction);
    }
  }
  return true;
}

}  // namespace routing
}  // namespace apollo

