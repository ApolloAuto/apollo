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

#include <algorithm>
#include <fstream>

#include "modules/common/log.h"
#include "modules/common/util/file.h"

#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/strategy/a_star_strategy.h"
#include "modules/routing/graph/sub_topo_graph.h"

namespace apollo {
namespace routing {

namespace {

bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph) {
  const auto& start = request.start();
  const auto& end = request.end();
  const auto* node = graph->GetNode(start.id());
  if (node == nullptr) {
     AERROR << "Start node is not found in topo graph! ID: " << start.id();
     return false;
  }
  AINFO << "Start point:\tlane id: " << start.id() << " s: " << start.s()
        << " x: " << start.pose().x() << " y: " << start.pose().y()
        << " length: " << node->Length();

  for (const auto& wp : request.waypoint()) {
    node = graph->GetNode(wp.id());
    if (node == nullptr) {
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      return false;
    }
    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
  }

  for (const auto& bl : request.blacklisted_lane()) {
    node = graph->GetNode(bl.id());
    if (node == nullptr) {
      AERROR << "Black list node is not found in topo graph! ID: "
          << bl.id();
      return false;
    }
    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
  }

  node = graph->GetNode(end.id());
  if (node == nullptr) {
    AERROR << "End node is not found in topo graph! ID: " << end.id();
    return false;
  }
  AINFO << "End point:\tlane id: " << end.id() << " s: " << end.s()
        << " x: " << end.pose().x() << " y: " << end.pose().y()
        << " length: " << node->Length();
  return true;
}

bool GetWayNodes(const RoutingRequest& request,
                 const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  const auto* start_node = graph->GetNode(request.start().id());
  if (start_node == nullptr) {
    AERROR << "Can't find start point in graph! Id: " << request.start().id();
    return false;
  }
  way_nodes->push_back(start_node);
  way_s->push_back(request.start().s());

  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
        AERROR << "Can't find way point in graph! Id: " << point.id();
        return false;
    }
    way_nodes->push_back(cur_node);
    way_s->push_back(point.s());
  }

  const auto* end_node = graph->GetNode(request.end().id());
  if (end_node == nullptr) {
    AERROR << "Can't find end point in graph! Id: " << request.end().id();
    return false;
  }
  way_nodes->push_back(end_node);
  way_s->push_back(request.end().s());
  return true;
}

void SetErrorCode(const RoutingResponse::ErrorCode::ErrorID& error_id,
                  const std::string& error_string,
                  RoutingResponse::ErrorCode* const error_code) {
  error_code->set_error_id(error_id);
  error_code->set_error_string(error_string);
  if (error_id == RoutingResponse::ErrorCode::SUCCESS) {
      AINFO << error_string.c_str();
  } else {
      AERROR << error_string.c_str();
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    AINFO << node.GetTopoNode()->LaneId() << "\t"
        << node.GetTopoNode()->IsVirtual() << "\t"
        << node.StartS() << "\t" << node.EndS();
  }
}

}  // namespace

Navigator::Navigator(const std::string& topo_file_path)
    : is_ready_(false) {
  Graph graph;
  if (!common::util::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    return;
  }

  graph_.reset(new TopoGraph());
  if (!graph_->LoadGraph(graph)) {
      AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
      return;
  }
  black_list_generator_.reset(new BlackListRangeGenerator);
  result_generator_.reset(new ResultGenerator);
  is_ready_ = true;
  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() { }

bool Navigator::IsReady() const {
  return is_ready_;
}

void Navigator::Clear() {
  topo_range_manager_.Clear();
}

bool Navigator::Init(const RoutingRequest& request,
                     const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  Clear();
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  black_list_generator_->GenerateBlackMapFromRequest(request,
                                                     graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  bool need_to_merge = false;
  for (size_t i = 0; i < node_vec.size(); ++i) {
    if (!need_to_merge) {
      result_node_vec->push_back(node_vec[i]);
    } else {
      if (result_node_vec->back().EndS() < node_vec[i].StartS()) {
        AERROR << "Result route is not coninuous";
        return false;
      }
      result_node_vec->back().SetEndS(node_vec[i].EndS());
    }
    if (i < node_vec.size() - 1) {
      need_to_merge = (node_vec[i].GetTopoNode()
                            == node_vec[i + 1].GetTopoNode());
    }
  }
  return true;
}

bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph,
    const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    TopoRangeManager full_range_manager = topo_range_manager_;
    black_list_generator_->AddBlackMapFromTerminal(way_start,
                                                   way_end,
                                                   way_start_s,
                                                   way_end_s,
                                                   &full_range_manager);

    SubTopoGraph sub_graph(full_range_manager.RangeMap());
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph,
                              &sub_graph,
                              start,
                              end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from "
            << start->LaneId() << " to " << end->LaneId();
      return false;
    }

    node_vec.insert(node_vec.end(),
                    cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }

  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(RoutingResponse::ErrorCode::ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_error_code());
    return false;
  }

  if (!IsReady()) {
    SetErrorCode(RoutingResponse::ErrorCode::ERROR_ROUTER_NOT_READY,
                 "Navigator is not ready!",
                 response->mutable_error_code());
    return false;
  }
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(RoutingResponse::ErrorCode::ERROR_ROUTER_NOT_READY,
                 "Failed to initialize navigator!",
                 response->mutable_error_code());
    return false;
  }

  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(),
                             way_nodes,
                             way_s,
                             &result_nodes)) {
    SetErrorCode(RoutingResponse::ErrorCode::ERROR_RESPONSE_FAILED,
                 "Failed to find route with request!",
                 response->mutable_error_code());
    return false;
  }
  result_nodes.front().SetStartS(request.start().s());
  result_nodes.back().SetEndS(request.end().s());

  if (!result_generator_->GeneratePassageRegion(graph_->MapVersion(),
                                                request,
                                                result_nodes,
                                                topo_range_manager_,
                                                response)) {
    SetErrorCode(RoutingResponse::ErrorCode::ERROR_RESPONSE_FAILED,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_error_code());
    return false;
  }
  SetErrorCode(RoutingResponse::ErrorCode::SUCCESS,
               "Success!",
               response->mutable_error_code());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo

