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

#include "modules/routing/core/navigator.h"

#include <assert.h>
#include <float.h>

#include <algorithm>
#include <fstream>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_node.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using ::apollo::common::util::SetProtoToASCIIFile;
using ::apollo::common::adapter::AdapterManager;

void GetNodesOfWaysBasedOnVirtual(
    const std::vector<const TopoNode*>& nodes,
    std::vector<std::vector<const TopoNode*>>* const nodes_of_ways) {
  AINFO << "Cut routing ways based on is_virtual.";
  assert(!nodes.empty());
  nodes_of_ways->clear();
  std::vector<const TopoNode*> nodes_of_way;
  nodes_of_way.push_back(nodes.at(0));
  bool last_is_virtual = nodes.at(0)->IsVirtual();
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    if ((*iter)->IsVirtual() != last_is_virtual) {
      nodes_of_ways->push_back(nodes_of_way);
      nodes_of_way.clear();
    }
    nodes_of_way.push_back(*iter);
    last_is_virtual = (*iter)->IsVirtual();
  }
  nodes_of_ways->push_back(nodes_of_way);
}

void GetNodesOfWays(
    const std::vector<const TopoNode*>& nodes,
    std::vector<std::vector<const TopoNode*>>* const nodes_of_ways) {
  AINFO << "Cut routing ways based on road id.";
  assert(!nodes.empty());
  nodes_of_ways->clear();
  std::vector<const TopoNode*> nodes_of_way;
  nodes_of_way.push_back(nodes.at(0));
  std::string last_road_id = nodes.at(0)->RoadId();
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    if ((*iter)->RoadId() != last_road_id) {
      nodes_of_ways->push_back(nodes_of_way);
      nodes_of_way.clear();
    }
    nodes_of_way.push_back(*iter);
    last_road_id = (*iter)->RoadId();
  }
  nodes_of_ways->push_back(nodes_of_way);
}

void ExtractBasicPassages(
    const std::vector<const TopoNode*>& nodes,
    std::vector<std::vector<const TopoNode*>>* const nodes_of_passages,
    std::vector<RoutingResponse_LaneChangeInfo::Type>* const
        lane_change_types) {
  assert(!nodes.empty());
  nodes_of_passages->clear();
  lane_change_types->clear();
  std::vector<const TopoNode*> nodes_of_passage;
  nodes_of_passage.push_back(nodes.at(0));
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    auto edge = (*(iter - 1))->GetOutEdgeTo(*iter);
    CHECK(edge) << "Get null pointer to edge";
    if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {
      nodes_of_passages->push_back(nodes_of_passage);
      nodes_of_passage.clear();
      if (edge->Type() == TET_LEFT) {
        lane_change_types->push_back(
            RoutingResponse_LaneChangeInfo::LEFT_FORWARD);
      } else {
        lane_change_types->push_back(
            RoutingResponse_LaneChangeInfo::RIGHT_FORWARD);
      }
    }
    nodes_of_passage.push_back(*iter);
  }
  nodes_of_passages->push_back(nodes_of_passage);
}

void PassageLaneIdsToPassageRegion(
    const std::vector<const TopoNode*>& nodes,
    NodeRangeManager* const range_manager,
    RoutingResponse_PassageRegion* const region) {
  for (const auto& node : nodes) {
    RoutingResponse_LaneSegment* seg = region->add_segment();
    seg->set_id(node->LaneId());
    NodeRange range = range_manager->GetNodeRange(node);
    // handle start and end on the same node, but start is later than end
    if (range.start_s > range.end_s) {
      seg->set_start_s(range.start_s);
      seg->set_end_s(node->Length());
      range_manager->SetNodeS(node, 0.0, range.end_s);
    } else {
      seg->set_start_s(range.start_s);
      seg->set_end_s(range.end_s);
    }
  }
}

double CalculateDistance(const std::vector<const TopoNode*>& nodes,
                         NodeRangeManager* const range_manager) {
  NodeRange range = range_manager->GetNodeRange(nodes.at(0));
  double distance = range.end_s - range.start_s;
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    auto edge = (*(iter - 1))->GetOutEdgeTo(*iter);
    if (edge->Type() != TET_FORWARD) {
      continue;
    }
    range = range_manager->GetNodeRange(*iter);
    distance += range.end_s - range.start_s;
  }
  return distance;
}

template <typename T>
void ShowRequestInfo(const T& request) {
  const auto& start = request.start();
  const auto& end = request.end();
  AERROR << "Start point - lane id: " << start.id() << " s " << start.s()
         << " x " << start.pose().x() << " y " << start.pose().y();

  for (const auto& wp : request.waypoint()) {
    AINFO << "Way Point - lane id: " << wp.id() << " s " << wp.s() << " x "
          << wp.pose().x() << " y " << wp.pose().y();
  }

  for (const auto& bl : request.blacklisted_lane()) {
    AINFO << "Way Point - lane id: " << bl.id() << " start_s " << bl.start_s()
          << " end_s " << bl.end_s();
  }

  AERROR << "End point - lane id: " << end.id() << " s " << end.s() << " x "
         << end.pose().x() << " y " << end.pose().y();
}

void GenerateBlackSetFromRoad(
    const RoutingRequest& request, const TopoGraph* graph,
    std::unordered_set<const TopoNode*>* const black_list) {
  for (const auto& road_id : request.blacklisted_road()) {
    graph->GetNodesByRoadId(road_id, black_list);
  }
}

template <typename T>
void GenerateBlackSetFromLane(
    const T& request, const TopoGraph* graph,
    std::unordered_set<const TopoNode*>* const black_list) {
  for (const auto& point : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(point.id());
    if (node == nullptr) {
      continue;
    }
    black_list->insert(node);
  }
}

template <typename T>
bool GetWayNodes(const T& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 NodeRangeManager* const range_manager) {
  const auto* start_node = graph->GetNode(request.start().id());
  const auto* end_node = graph->GetNode(request.end().id());
  if (start_node == nullptr) {
    AERROR << "Can't find start point in graph! Id: " << request.start().id();
    return false;
  }
  if (end_node == nullptr) {
    AERROR << "Can't find end point in graph! Id: " << request.start().id();
    return false;
  }

  way_nodes->push_back(start_node);

  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Can't find way point in graph! Id: " << point.id();
      return false;
    }
    way_nodes->push_back(cur_node);
  }

  if (way_nodes->size() == 1 && start_node == end_node &&
      request.start().s() > request.end().s()) {
    if (start_node->OutToSucEdge().size() == 1) {
      const auto inter_edge = start_node->OutToSucEdge().begin();
      way_nodes->push_back((*inter_edge)->ToNode());
    } else if (end_node->InFromPreEdge().size() == 1) {
      const auto inter_edge = end_node->InFromPreEdge().begin();
      way_nodes->push_back((*inter_edge)->FromNode());
    } else if (start_node->OutToSucEdge().size() > 1) {
      const auto inter_edge = start_node->OutToSucEdge().begin();
      way_nodes->push_back((*inter_edge)->ToNode());
    } else if (end_node->InFromPreEdge().size() > 1) {
      const auto inter_edge = end_node->InFromPreEdge().begin();
      way_nodes->push_back((*inter_edge)->FromNode());
    } else {
      // on the same not but able to expand
      return false;
    }
  }

  way_nodes->push_back(end_node);

  range_manager->InitNodeRange(request.start().s(), request.end().s(),
                               start_node, end_node);

  return true;
}

bool SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::unordered_set<const TopoNode*>& black_list,
    std::vector<const TopoNode*>* const result_nodes) {
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy());

  result_nodes->clear();
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    std::vector<const TopoNode*> cur_result_nodes;
    const auto* cur_start = way_nodes[i - 1];
    const auto* cur_end = way_nodes[i];
    if (!strategy_ptr->Search(graph, cur_start, cur_end, black_list,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from "
             << cur_start->LaneId() << " to " << cur_end->LaneId();
      return false;
    }
    auto end_iter = cur_result_nodes.end();
    if (i != way_nodes.size() - 1) {
      --end_iter;
    }
    result_nodes->insert(result_nodes->end(), cur_result_nodes.begin(),
                         end_iter);
  }
  return true;
}

}  // namespace

Navigator::Navigator(const std::string& topo_file_path) : is_ready_(false) {
  graph_.reset(new TopoGraph());
  if (!graph_->LoadGraph(topo_file_path)) {
    AERROR << "Navigator init graph failed! File path: " << topo_file_path;
    return;
  }
  is_ready_ = true;
  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const {
  return is_ready_;
}

bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* response) const {
  if (!IsReady()) {
    AERROR << "Topo graph is not ready!";
    return false;
  }
  ShowRequestInfo(request);

  std::vector<const TopoNode*> way_nodes;
  NodeRangeManager range_manager;

  if (!GetWayNodes(request, graph_.get(), &way_nodes, &range_manager)) {
    AERROR << "Can't find way point in graph!";
    return false;
  }

  std::vector<const TopoNode*> result_nodes;
  std::unordered_set<const TopoNode*> black_list;
  GenerateBlackSetFromLane(request, graph_.get(), &black_list);
  GenerateBlackSetFromRoad(request, graph_.get(), &black_list);

  if (!SearchRouteByStrategy(graph_.get(), way_nodes, black_list,
                             &result_nodes)) {
    AERROR << "Can't find route from request!";
    return false;
  }

  if (!GeneratePassageRegion(request, result_nodes, black_list,
                             &range_manager, response)) {
    AERROR
        << "Failed to generate new passage regions based on route result lane";
    return false;
  }

  if (FLAGS_enable_debug_mode) {
    DumpDebugData(result_nodes, range_manager, *response);
  }
  return true;
}

// new request to new response
bool Navigator::GeneratePassageRegion(
    const RoutingRequest& request,
    const std::vector<const TopoNode*>& nodes,
    const std::unordered_set<const TopoNode*>& black_list,
    NodeRangeManager* const range_manager,
    RoutingResponse* result) const {
  AdapterManager::FillRoutingResponseHeader(FLAGS_node_name, result);

  GeneratePassageRegion(nodes, black_list, range_manager, result);

  result->set_map_version(graph_->MapVersion());
  result->mutable_measurement()->set_distance(
      CalculateDistance(nodes, range_manager));
  result->mutable_routing_request()->CopyFrom(request);
  return true;
}

// use internal generate result
void Navigator::GeneratePassageRegion(
    const std::vector<const TopoNode*>& nodes,
    const std::unordered_set<const TopoNode*>& black_list,
    NodeRangeManager* const range_manager,
    RoutingResponse* result) const {
  std::vector<std::vector<const TopoNode*>> nodes_of_ways;
  if (FLAGS_use_road_id) {
    GetNodesOfWays(nodes, &nodes_of_ways);
  } else {
    GetNodesOfWaysBasedOnVirtual(nodes, &nodes_of_ways);
  }
  size_t num_of_roads = 0;
  size_t num_of_junctions = 0;
  for (size_t i = 0; i < nodes_of_ways.size(); ++i) {
    AINFO << "Way " << std::to_string(i);
    const std::vector<const TopoNode*>& nodes_of_way = nodes_of_ways.at(i);
    if (FLAGS_use_road_id || !nodes_of_way.at(0)->IsVirtual()) {
      std::vector<std::vector<const TopoNode*>> nodes_of_basic_passages;
      std::vector<RoutingResponse_LaneChangeInfo::Type> lane_change_types;
      ExtractBasicPassages(nodes_of_way, &nodes_of_basic_passages,
                           &lane_change_types);

      RoutingResponse_Road road;
      road.set_id("r" + std::to_string(num_of_roads));
      auto node = nodes_of_basic_passages.front().front();
      road.mutable_in_lane()->set_id(node->LaneId());
      road.mutable_in_lane()->set_s(range_manager->GetNodeStartS(node));
      node = nodes_of_basic_passages.back().back();
      road.mutable_out_lane()->set_id(node->LaneId());
      road.mutable_out_lane()->set_s(range_manager->GetNodeEndS(node));
      for (const auto& nodes_of_passage : nodes_of_basic_passages) {
        PassageLaneIdsToPassageRegion(nodes_of_passage, range_manager,
                                      road.add_passage_region());
      }
      for (size_t i = 0; i < lane_change_types.size(); ++i) {
        RoutingResponse_LaneChangeInfo* lc_info = road.add_lane_change_info();
        lc_info->set_type(lane_change_types.at(i));
        lc_info->set_start_passage_region_index(i);
        lc_info->set_end_passage_region_index(i + 1);
      }
      result->add_route()->mutable_road_info()->CopyFrom(road);
      num_of_roads++;
    } else {
      RoutingResponse_Junction junction;
      RoutingResponse_PassageRegion region;
      junction.set_id("j" + std::to_string(num_of_junctions));
      junction.set_in_road_id("r" + std::to_string(num_of_roads - 1));
      junction.set_out_road_id("r" + std::to_string(num_of_roads));
      for (const auto& node : nodes_of_way) {
        RoutingResponse_LaneSegment* seg = region.add_segment();
        seg->set_id(node->LaneId());
        NodeRange range = range_manager->GetNodeRange(node);
        seg->set_start_s(range.start_s);
        seg->set_end_s(range.end_s);
      }
      junction.mutable_passage_region()->CopyFrom(region);
      result->add_route()->mutable_junction_info()->CopyFrom(junction);
      AINFO << "Junction passage!!!";
      num_of_junctions++;
    }
  }
}

void Navigator::DumpDebugData(
    const std::vector<const TopoNode*>& nodes,
    const NodeRangeManager& range_manager,
    const RoutingResponse& response) const {
  std::ofstream fout(FLAGS_debug_route_path);
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    NodeRange range = range_manager.GetNodeRange(node);
    fout << node->LaneId() << ", " << node->IsVirtual() << ","
         << range.start_s << "," << range.end_s << "\n";
    AINFO << node->LaneId() << "\t" << node->IsVirtual() << range.start_s
          << "\t" << range.end_s;
  }

  fout.close();

  std::string dump_path = FLAGS_debug_passage_region_path;
  if (!SetProtoToASCIIFile(response, dump_path)) {
    AERROR << "Failed to dump passage region debug file.";
  }
  AINFO << "Passage region debug file is dumped successfully. Dump path: "
        << dump_path;
}

}  // namespace routing
}  // namespace apollo
