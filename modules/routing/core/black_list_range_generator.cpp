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
#include "ros/ros.h"

#include "core/black_list_range_generator.h"

#include "graph/sub_topo_graph.h"
#include "graph/topo_graph.h"
#include "graph/topo_node.h"

namespace adu {
namespace routing {

namespace {

NodeWithRange get_node_with_range(const TopoNode* node, double start_s, double end_s) {
    if (node == nullptr) {
        ROS_ERROR("Get null pointer to topo node when getting node with range.");
        exit(-1);
    }
    if (start_s < 0.0 || end_s > node->length()) {
        ROS_ERROR("Invalid s, start_s: %f, end_s: %f, length: %f.", start_s, end_s, node->length());
        exit(-1);
    }
    NodeSRange range(start_s, end_s);
    NodeWithRange black_node_with_range(range, node);
    return black_node_with_range;
}

void generate_black_set_from_road(const ::adu::common::router::RoutingRequest& request,
                                  const TopoGraph* graph,
                                  std::unordered_set<const TopoNode*>* const black_list) {
    for (const auto& road_id : request.blacklisted_road()) {
        graph->get_nodes_by_road_id(road_id, black_list);
    }
}

void generate_black_set_from_road(const ::adu::common::router::RoutingRequest& request,
                                  const TopoGraph* graph,
                                  std::vector<NodeWithRange>* const black_list) {
    for (const auto& road_id : request.blacklisted_road()) {
        std::unordered_set<const TopoNode*> road_nodes_set;
        graph->get_nodes_by_road_id(road_id, &road_nodes_set);
        for (const auto& node : road_nodes_set) {
            black_list->push_back(get_node_with_range(node, 0.0, node->length()));
        }
    }
}

template <typename T>
void generate_black_set_from_lane(const T& request,
                                  const TopoGraph* graph,
                                  std::unordered_set<const TopoNode*>* const black_list) {
    for (const auto& lane : request.blacklisted_lane()) {
        const auto* node = graph->get_node(lane.id());
        if (node == nullptr) {
            continue;
        }
        black_list->insert(node);
    }
}

template <typename T>
void generate_black_set_from_lane(const T& request,
                                  const TopoGraph* graph,
                                  std::vector<NodeWithRange>* const black_list) {
    for (const auto& lane : request.blacklisted_lane()) {
        const auto* node = graph->get_node(lane.id());
        if (node == nullptr) {
            continue;
        }
        black_list->push_back(get_node_with_range(node, lane.start_s(), lane.end_s()));
    }
}

} // namespace

// old request
void BlackListRangeGenerator::generate_black_set_from_request(
                                    const ::adu::common::routing::RoutingRequest& request,
                                    const TopoGraph* graph,
                                    std::unordered_set<const TopoNode*>* const black_list) const {
    generate_black_set_from_lane(request, graph, black_list);
}

// new request
void BlackListRangeGenerator::generate_black_set_from_request(
                                    const ::adu::common::router::RoutingRequest& request,
                                    const TopoGraph* graph,
                                    std::unordered_set<const TopoNode*>* const black_list) const {
    generate_black_set_from_lane(request, graph, black_list);
    generate_black_set_from_road(request, graph, black_list);
}

// old request
void BlackListRangeGenerator::generate_black_vec_from_request(
                                    const ::adu::common::routing::RoutingRequest& request,
                                    const TopoGraph* graph,
                                    const TopoNode* src_node,
                                    const TopoNode* dest_node,
                                    std::vector<NodeWithRange>* const black_list) const {
    generate_black_set_from_lane(request, graph, black_list);
    double start_s = request.start().s();
    double end_s = request.end().s();
    if (src_node == dest_node && start_s > end_s) {
        black_list->push_back(get_node_with_range(src_node, end_s, start_s));
    } else {
        NodeSRange range(end_s, start_s);
        NodeWithRange black_node_with_range(range, src_node);
        black_list->push_back(black_node_with_range);

    }
 
        

}

    // new request
void BlackListRangeGenerator::generate_black_vec_from_request(
                                    const ::adu::common::router::RoutingRequest& request,
                                    const TopoGraph* graph,
                                    const TopoNode* src_node,
                                    const TopoNode* dest_node,
                                    std::vector<NodeWithRange>* const black_list) const {
    generate_black_set_from_lane(request, graph, black_list);
    generate_black_set_from_road(request, graph, black_list);
}

} // namespace routing
} // namespace adu
