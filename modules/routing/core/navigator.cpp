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
#include <fstream>
#include <algorithm>

#include "ros/console.h"
#include "ros/time.h"
#include "boost/lexical_cast.hpp"

#include "common/routing_gflags.h"
#include "common/utils.h"
#include "strategy/a_star_strategy.h"
#include "core/black_list_range_generator.h"
#include "core/routing_result_generator.h"
#include "core/router_result_generator.h"
#include "graph/sub_topo_graph.h"
#include "graph/topo_graph.h"
#include "graph/topo_edge.h"
#include "graph/topo_node.h"

namespace adu {
namespace routing {

namespace {

template <typename T>
void show_request_info(const T& request) {
    const auto& start = request.start();
    const auto& end = request.end();
    ROS_INFO("Start point:\tlane id: %s s: %f x: %f y: %f",
             start.id().c_str(),
             start.s(),
             start.pose().x(),
             start.pose().y());

    for (const auto& wp : request.waypoint()) {
        ROS_INFO("Way Point:\tlane id: %s s: %f x: %f y: %f",
                wp.id().c_str(),
                wp.s(),
                wp.pose().x(),
                wp.pose().y());
    }

    for (const auto& bl : request.blacklisted_lane()) {
        ROS_INFO("Way Point:\tlane id: %s start_s: %f end_s %f",
                bl.id().c_str(),
                bl.start_s(),
                bl.end_s());
    }

    ROS_INFO("End point:\tlane id: %s s: %f x: %f y: %f",
             end.id().c_str(),
             end.s(),
             end.pose().x(),
             end.pose().y());
}

template <typename T>
bool get_way_nodes(const T& request,
                   const TopoGraph* graph,
                   std::vector<const TopoNode*>* const way_nodes,
                   NodeSRangeManager* const range_manager) {
    const auto* start_node = graph->get_node(request.start().id());
    if (start_node == nullptr) {
        ROS_ERROR("Can't find start point in graph! Id: %s", request.start().id().c_str());
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
        ROS_ERROR("Can't find end point in graph! Id: %s", request.start().id().c_str());
        return false;
    }
    way_nodes->push_back(end_node);

    range_manager->init_node_range(request.start().s(), request.end().s(), start_node, end_node);

    return true;
}

bool search_route_by_strategy(const TopoGraph* graph,
                              const std::vector<const TopoNode*>& way_nodes,
                              const std::unordered_set<const TopoNode*>& black_list,
                              std::vector<const TopoNode*>* const result_nodes) {
    std::unique_ptr<Strategy> strategy_ptr;
    strategy_ptr.reset(new AStarStrategy());

    result_nodes->clear();
    for (size_t i = 1; i < way_nodes.size(); ++i) {
        std::vector<const TopoNode*> cur_result_nodes;
        const auto* cur_start = way_nodes[i - 1];
        const auto* cur_end = way_nodes[i];
        if (!strategy_ptr->search(graph, cur_start, cur_end, black_list, &cur_result_nodes)) {
            ROS_ERROR("Failed to search route with waypoint from %s to %s",
                    cur_start->lane_id().c_str(), cur_end->lane_id().c_str());
            return false;
        }
        auto end_iter = cur_result_nodes.end();
        if (i != way_nodes.size() - 1) {
            --end_iter;
        }
        result_nodes->insert(result_nodes->end(), cur_result_nodes.begin(), end_iter);
    }
    return true;
}

} // namespace

Navigator::Navigator(const std::string& topo_file_path) : _is_ready(false) {
    ::adu::routing::common::Graph graph;
    if (!FileUtils::load_protobuf_data_from_file(topo_file_path, &graph)) {
        ROS_ERROR("Failed to read topology graph from data.");
        return;
    }

    _graph.reset(new TopoGraph());
    if (!_graph->load_graph(graph)) {
        ROS_INFO("Navigator init graph failed! File path: %s.", topo_file_path.c_str());
        return;
    }
    _black_list_generator.reset(new BlackListRangeGenerator);
    _routing_result_generator.reset(new RoutingResultGenerator);
    _router_result_generator.reset(new RouterResultGenerator);
    _is_ready = true;
    ROS_INFO("The navigator is ready.");
};

Navigator::~Navigator() { }

bool Navigator::is_ready() const {
    return _is_ready;
}

// search new request to new response
bool Navigator::search_route(const ::adu::common::router::RoutingRequest& request,
                             ::adu::common::router::RoutingResult* response) const {
    if (!is_ready()) {
        ROS_ERROR("Topo graph is not ready!");
        return false;
    }
    show_request_info(request);

    std::vector<const TopoNode*> way_nodes;
    NodeSRangeManager range_manager;

    if (!get_way_nodes(request, _graph.get(), &way_nodes, &range_manager)) {
        ROS_ERROR("Can't find way point in graph!");
        return false;
    }

    std::vector<const TopoNode*> result_nodes;
    std::unordered_set<const TopoNode*> black_list;
    _black_list_generator->generate_black_set_from_request(request, _graph.get(), &black_list);

    if (!search_route_by_strategy(_graph.get(), way_nodes, black_list, &result_nodes)) {
        ROS_ERROR("Can't find route from request!");
        return false;
    }

    if (!_router_result_generator->generate_passage_region(_graph->map_version(),
                                                           request,
                                                           result_nodes,
                                                           black_list,
                                                           range_manager,
                                                           response)) {
        ROS_ERROR("Failed to generate new passage regions based on route result lane ids");
        return false;
    }

    if (FLAGS_enable_debug_mode) {
        dump_debug_data(result_nodes, range_manager, *response);
    }
    return true;
}

// search old request to old response
bool Navigator::search_route(const ::adu::common::routing::RoutingRequest& request,
                             ::adu::common::routing::RoutingResult* response) const {
    if (!is_ready()) {
        ROS_ERROR("Topo graph is not ready!");
        return false;
    }
    show_request_info(request);

    std::vector<const TopoNode*> way_nodes;
    NodeSRangeManager range_manager;

    if (!get_way_nodes(request, _graph.get(), &way_nodes, &range_manager)) {
        ROS_ERROR("Can't find way point in graph!");
        return false;
    }

    std::vector<const TopoNode*> result_nodes;
    std::unordered_set<const TopoNode*> black_list;
    _black_list_generator->generate_black_set_from_request(request, _graph.get(), &black_list);

    if (!search_route_by_strategy(_graph.get(), way_nodes, black_list, &result_nodes)) {
        ROS_ERROR("Can't find route from request!");
        return false;
    }

    if (!_routing_result_generator->generate_passage_region(request,
                                                            result_nodes,
                                                            black_list,
                                                            range_manager,
                                                            response)) {
        ROS_ERROR("Failed to generate new passage regions based on route result lane ids");
        return false;
    }

    if (FLAGS_enable_debug_mode) {
        ::adu::common::router::RoutingResult new_response;
        _router_result_generator->generate_passage_region(result_nodes,
                                                          black_list,
                                                          range_manager,
                                                          &new_response);
        dump_debug_data(result_nodes, range_manager, new_response);
    }

    return true;
}

void Navigator::dump_debug_data(const std::vector<const TopoNode*>& nodes,
                                const NodeSRangeManager& range_manager,
                                const ::adu::common::router::RoutingResult& response) const {
    std::string debug_string;
    ROS_INFO("Route lane id\tis virtual\tstart s\tend s");
    for (const auto& node : nodes) {
        NodeSRange range = range_manager.get_node_range(node);
        debug_string += node->lane_id() + ","
            + boost::lexical_cast<std::string>(node->is_virtual()) + ","
            + boost::lexical_cast<std::string>(range.start_s()) + ","
            + boost::lexical_cast<std::string>(range.end_s()) + "\n";
        ROS_INFO("%s\t%d\t%.2f\t%.2f", node->lane_id().c_str(),
                                       node->is_virtual(),
                                       range.start_s(),
                                       range.end_s());
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

} // namespace routing
} // namespace adu

