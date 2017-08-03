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

#include "core/navigator.h"

#include <assert.h>
#include <algorithm>
#include <fstream>

#include "boost/lexical_cast.hpp"
#include "ros/console.h"
#include "ros/time.h"

#include <float.h>
#include "common/routing_gflags.h"
#include "common/utils.h"
#include "graph/topo_edge.h"
#include "graph/topo_graph.h"
#include "graph/topo_node.h"
#include "strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using ::apollo::routing::RoutingResponse_Road;
using ::apollo::routing::RoutingResponse_Junction;
using ::apollo::routing::RoutingResponse_PassageRegion;
using ::apollo::routing::RoutingResponse_LaneSegment;
using ::apollo::routing::RoutingResponse_LaneChangeInfo;

void get_nodes_of_ways_based_on_virtual(
    const std::vector<const TopoNode*>& nodes,
    std::vector<std::vector<const TopoNode*> >* const nodes_of_ways) {
  ROS_INFO("Cut routing ways based on is_virtual.");
  assert(!nodes.empty());
  nodes_of_ways->clear();
  std::vector<const TopoNode*> nodes_of_way;
  nodes_of_way.push_back(nodes.at(0));
  bool last_is_virtual = nodes.at(0)->is_virtual();
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    if ((*iter)->is_virtual() != last_is_virtual) {
      nodes_of_ways->push_back(nodes_of_way);
      nodes_of_way.clear();
    }
    nodes_of_way.push_back(*iter);
    last_is_virtual = (*iter)->is_virtual();
  }
  nodes_of_ways->push_back(nodes_of_way);
}

void get_nodes_of_ways(
    const std::vector<const TopoNode*>& nodes,
    std::vector<std::vector<const TopoNode*> >* const nodes_of_ways) {
  ROS_INFO("Cut routing ways based on road id.");
  assert(!nodes.empty());
  nodes_of_ways->clear();
  std::vector<const TopoNode*> nodes_of_way;
  nodes_of_way.push_back(nodes.at(0));
  std::string last_road_id = nodes.at(0)->road_id();
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    if ((*iter)->road_id() != last_road_id) {
      nodes_of_ways->push_back(nodes_of_way);
      nodes_of_way.clear();
    }
    nodes_of_way.push_back(*iter);
    last_road_id = (*iter)->road_id();
  }
  nodes_of_ways->push_back(nodes_of_way);
}

void extract_basic_passages(
    const std::vector<const TopoNode*>& nodes,
    std::vector<std::vector<const TopoNode*> >* const nodes_of_passages,
    std::vector<RoutingResponse_LaneChangeInfo::Type>* const lane_change_types) {
  assert(!nodes.empty());
  nodes_of_passages->clear();
  lane_change_types->clear();
  std::vector<const TopoNode*> nodes_of_passage;
  nodes_of_passage.push_back(nodes.at(0));
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    auto edge = (*(iter - 1))->get_out_edge_to(*iter);
    if (edge == nullptr) {
      ROS_ERROR("Get null pointer to edge from %s to %s.",
                (*(iter - 1))->lane_id().c_str(), (*iter)->lane_id().c_str());
      exit(-1);
    }
    if (edge->type() == TET_LEFT || edge->type() == TET_RIGHT) {
      nodes_of_passages->push_back(nodes_of_passage);
      nodes_of_passage.clear();
      if (edge->type() == TET_LEFT) {
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

void passage_lane_ids_to_passage_region(
    const std::vector<const TopoNode*>& nodes,
    const NodeRangeManager& range_manager,
    RoutingResponse_PassageRegion* const region) {
  for (const auto& node : nodes) {
    RoutingResponse_LaneSegment* seg = region->add_segment();
    seg->set_id(node->lane_id());
    NodeRange range = range_manager.get_node_range(node);
    seg->set_start_s(range.start_s);
    seg->set_end_s(range.end_s);
  }
}

double calculate_distance(const std::vector<const TopoNode*>& nodes,
                          const NodeRangeManager& range_manager) {
  NodeRange range = range_manager.get_node_range(nodes.at(0));
  double distance = range.end_s - range.start_s;
  for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
    auto edge = (*(iter - 1))->get_out_edge_to(*iter);
    if (edge->type() != TET_FORWARD) {
      continue;
    }
    range = range_manager.get_node_range(*iter);
    distance += range.end_s - range.start_s;
  }
  return distance;
}

template <typename T>
void show_request_info(const T& request) {
  const auto& start = request.start();
  const auto& end = request.end();
  ROS_INFO("Start point:\tlane id: %s s: %f x: %f y: %f", start.id().c_str(),
           start.s(), start.pose().x(), start.pose().y());

  for (const auto& wp : request.waypoint()) {
    ROS_INFO("Way Point:\tlane id: %s s: %f x: %f y: %f", wp.id().c_str(),
             wp.s(), wp.pose().x(), wp.pose().y());
  }

  for (const auto& bl : request.blacklisted_lane()) {
    ROS_INFO("Way Point:\tlane id: %s start_s: %f end_s %f", bl.id().c_str(),
             bl.start_s(), bl.end_s());
  }

  ROS_INFO("End point:\tlane id: %s s: %f x: %f y: %f", end.id().c_str(),
           end.s(), end.pose().x(), end.pose().y());
}

void generate_black_set_from_road(
    const ::apollo::routing::RoutingRequest& request,
    const TopoGraph* graph,
    std::unordered_set<const TopoNode*>* const black_list) {
  for (const auto& road_id : request.blacklisted_road()) {
    graph->get_nodes_by_road_id(road_id, black_list);
  }
}

template <typename T>
void generate_black_set_from_lane(
    const T& request, const TopoGraph* graph,
    std::unordered_set<const TopoNode*>* const black_list) {
  for (const auto& point : request.blacklisted_lane()) {
    const auto* node = graph->get_node(point.id());
    if (node == nullptr) {
      continue;
    }
    black_list->insert(node);
  }
}

template <typename T>
bool get_way_nodes(const T& request, const TopoGraph* graph,
                   std::vector<const TopoNode*>* const way_nodes,
                   NodeRangeManager* const range_manager) {
  const auto* start_node = graph->get_node(request.start().id());
  if (start_node == nullptr) {
    ROS_ERROR("Can't find start point in graph! Id: %s",
              request.start().id().c_str());
    return false;
  }
  way_nodes->push_back(start_node);

  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->get_node(point.id());
    if (cur_node == nullptr) {
      ROS_ERROR("Can't find way point in graph! Id: %s", point.id().c_str());
      return false;
    }
    way_nodes->push_back(cur_node);
  }

  const auto* end_node = graph->get_node(request.end().id());
  if (end_node == nullptr) {
    ROS_ERROR("Can't find end point in graph! Id: %s",
              request.start().id().c_str());
    return false;
  }
  way_nodes->push_back(end_node);

  range_manager->init_node_range(request.start().s(), request.end().s(),
                                 start_node, end_node);

  return true;
}

bool search_route_by_strategy(
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
    if (!strategy_ptr->search(graph, cur_start, cur_end, black_list,
                              &cur_result_nodes)) {
      ROS_ERROR("Failed to search route with waypoint from %s to %s",
                cur_start->lane_id().c_str(), cur_end->lane_id().c_str());
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

Navigator::Navigator(const std::string& topo_file_path) : _is_ready(false) {
  _graph.reset(new TopoGraph());
  if (!_graph->load_graph(topo_file_path)) {
    ROS_INFO("Navigator init graph failed! File path: %s.",
             topo_file_path.c_str());
    return;
  }
  _is_ready = true;
  ROS_INFO("The navigator is ready.");
};

Navigator::~Navigator() {}

bool Navigator::is_ready() const { return _is_ready; }

// search new request to new response
bool Navigator::search_route(
    const ::apollo::routing::RoutingRequest& request,
    ::apollo::routing::RoutingResponse* response) const {
  if (!is_ready()) {
    ROS_ERROR("Topo graph is not ready!");
    return false;
  }
  show_request_info(request);

  std::vector<const TopoNode*> way_nodes;
  NodeRangeManager range_manager;

  if (!get_way_nodes(request, _graph.get(), &way_nodes, &range_manager)) {
    ROS_ERROR("Can't find way point in graph!");
    return false;
  }

  std::vector<const TopoNode*> result_nodes;
  std::unordered_set<const TopoNode*> black_list;
  generate_black_set_from_lane(request, _graph.get(), &black_list);
  generate_black_set_from_road(request, _graph.get(), &black_list);

  if (!search_route_by_strategy(_graph.get(), way_nodes, black_list,
                                &result_nodes)) {
    ROS_ERROR("Can't find route from request!");
    return false;
  }

  if (!generate_passage_region(request, result_nodes, black_list, range_manager,
                               response)) {
    ROS_ERROR(
        "Failed to generate new passage regions based on route result lane "
        "ids");
    return false;
  }

  if (FLAGS_enable_debug_mode) {
    dump_debug_data(result_nodes, range_manager, *response);
  }
  return true;
}

// new request to new response
bool Navigator::generate_passage_region(
    const ::apollo::routing::RoutingRequest& request,
    const std::vector<const TopoNode*>& nodes,
    const std::unordered_set<const TopoNode*>& black_list,
    const NodeRangeManager& range_manager,
    ::apollo::routing::RoutingResponse* result) const {
  result->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  result->mutable_header()->set_module_name(FLAGS_node_name);
  result->mutable_header()->set_sequence_num(1);

  generate_passage_region(nodes, black_list, range_manager, result);

  result->set_map_version(_graph->map_version());
  result->mutable_measurement()->set_distance(
      calculate_distance(nodes, range_manager));
  result->mutable_routing_request()->CopyFrom(request);
  return true;
}

// use internal generate result
void Navigator::generate_passage_region(
    const std::vector<const TopoNode*>& nodes,
    const std::unordered_set<const TopoNode*>& black_list,
    const NodeRangeManager& range_manager,
    ::apollo::routing::RoutingResponse* result) const {
  std::vector<std::vector<const TopoNode*> > nodes_of_ways;
  if (FLAGS_use_road_id) {
    get_nodes_of_ways(nodes, &nodes_of_ways);
  } else {
    get_nodes_of_ways_based_on_virtual(nodes, &nodes_of_ways);
  }
  size_t num_of_roads = 0;
  size_t num_of_junctions = 0;
  for (size_t i = 0; i < nodes_of_ways.size(); ++i) {
    ROS_INFO("-----------------Way %d-----------------", int(i));
    const std::vector<const TopoNode*>& nodes_of_way = nodes_of_ways.at(i);
    if (!nodes_of_way.at(0)->is_virtual()) {
      std::vector<std::vector<const TopoNode*> > nodes_of_basic_passages;
      std::vector<RoutingResponse_LaneChangeInfo::Type> lane_change_types;
      extract_basic_passages(nodes_of_way, &nodes_of_basic_passages,
                             &lane_change_types);

      RoutingResponse_Road road;
      road.set_id("r" + boost::lexical_cast<std::string>(num_of_roads));
      auto node = nodes_of_basic_passages.front().front();
      road.mutable_in_lane()->set_id(node->lane_id());
      road.mutable_in_lane()->set_s(range_manager.get_node_start_s(node));
      node = nodes_of_basic_passages.back().back();
      road.mutable_out_lane()->set_id(node->lane_id());
      road.mutable_out_lane()->set_s(range_manager.get_node_end_s(node));
      for (const auto& nodes_of_passage : nodes_of_basic_passages) {
        passage_lane_ids_to_passage_region(nodes_of_passage, range_manager,
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
      junction.set_id("j" + boost::lexical_cast<std::string>(num_of_junctions));
      junction.set_in_road_id(
          "r" + boost::lexical_cast<std::string>(num_of_roads - 1));
      junction.set_out_road_id("r" +
                               boost::lexical_cast<std::string>(num_of_roads));
      RoutingResponse_PassageRegion region;
      for (const auto& node : nodes_of_way) {
        RoutingResponse_LaneSegment* seg = region.add_segment();
        seg->set_id(node->lane_id());
        NodeRange range = range_manager.get_node_range(node);
        seg->set_start_s(range.start_s);
        seg->set_end_s(range.end_s);
      }
      junction.mutable_passage_region()->CopyFrom(region);
      result->add_route()->mutable_junction_info()->CopyFrom(junction);
      ROS_INFO("Junction passage!!!");
      for (const auto& node : nodes_of_way) {
        ROS_INFO("lane id: %s", node->lane_id().c_str());
      }
      num_of_junctions++;
    }
  }
}

void Navigator::dump_debug_data(
    const std::vector<const TopoNode*>& nodes,
    const NodeRangeManager& range_manager,
    const ::apollo::routing::RoutingResponse& response) const {
  std::string debug_string;
  ROS_INFO("Route lane id\tis virtual\tstart s\tend s");
  for (const auto& node : nodes) {
    NodeRange range = range_manager.get_node_range(node);
    debug_string += node->lane_id() + "," +
                    boost::lexical_cast<std::string>(node->is_virtual()) + "," +
                    boost::lexical_cast<std::string>(range.start_s) + "," +
                    boost::lexical_cast<std::string>(range.end_s) + "\n";
    ROS_INFO("%s\t%d\t%.2f\t%.2f", node->lane_id().c_str(), node->is_virtual(),
             range.start_s, range.end_s);
  }

  std::ofstream fout(FLAGS_debug_route_path);
  fout << debug_string;
  fout.close();

  std::string dump_path = FLAGS_debug_passage_region_path;
  if (!FileUtils::dump_protobuf_data_to_file(dump_path, response)) {
    ROS_ERROR("Failed to dump passage region debug file.");
  }
  ROS_INFO("Passage region debug file is dumped successfully. Dump path: %s.",
           dump_path.c_str());
}

}  // namespace routing
}  // namespace apollo
