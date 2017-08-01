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

#include "core/router_result_generator.h"

#include "ros/console.h"
#include "ros/time.h"
#include "boost/lexical_cast.hpp"

#include "common/routing_gflags.h"
#include "graph/topo_node.h"
#include "graph/topo_edge.h"

namespace adu {
namespace routing {

namespace {

using ::adu::common::router::RoutingRequest;
using ::adu::common::router::RoutingResult;
using ::adu::common::router::RoutingResult_Road;
using ::adu::common::router::RoutingResult_Junction;
using ::adu::common::router::RoutingResult_PassageRegion;
using ::adu::common::router::RoutingResult_LaneSegment;
using ::adu::common::router::RoutingResult_LaneChangeInfo;

/*const double MAX_ALLOWED_DISTANCE = 100;
double node2nodes_min_distance(const TopoNode* src_node,
    const std::vector<const TopoNode*>& dest_nodes) {
    double min_dis = std::numeric_limits<double>::max();
    const ::adu::common::hdmap::Point& src_point = src_node->anchor_point();
    for (const auto& dest_node : dest_nodes) {
        const ::adu::common::hdmap::Point& dest_point = dest_node->anchor_point();
        double distance = sqrt(pow(src_point.x() - dest_point.x(), 2)
                + pow(src_point.y() - dest_point.y(), 2)
                + pow(src_point.z() - dest_point.z(), 2));
        min_dis = (distance < min_dis) ? distance : min_dis;
    }
    return min_dis;
}*/

void get_nodes_of_ways_based_on_virtual(const std::vector<const TopoNode*>& nodes,
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

void get_nodes_of_ways(const std::vector<const TopoNode*>& nodes,
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

void extract_basic_passages(const std::vector<const TopoNode*>& nodes,
                            std::vector<std::vector<const TopoNode*> >* const nodes_of_passages,
                            std::vector<RoutingResult_LaneChangeInfo::Type>* const
                                lane_change_types) {
    assert(!nodes.empty());
    nodes_of_passages->clear();
    lane_change_types->clear();
    std::vector<const TopoNode*> nodes_of_passage;
    nodes_of_passage.push_back(nodes.at(0));
    for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
        auto edge = (*(iter - 1))->get_out_edge_to(*iter);
        if (edge == nullptr) {
            ROS_ERROR("Get null pointer to edge from %s to %s.", (*(iter - 1))->lane_id().c_str(),
                                                                 (*iter)->lane_id().c_str());
            exit(-1);
        }
        if (edge->type() == TET_LEFT || edge->type() == TET_RIGHT) {
            nodes_of_passages->push_back(nodes_of_passage);
            nodes_of_passage.clear();
            if (edge->type() == TET_LEFT) {
                lane_change_types->push_back(RoutingResult_LaneChangeInfo::LEFT_FORWARD);
            } else {
                lane_change_types->push_back(RoutingResult_LaneChangeInfo::RIGHT_FORWARD);
            }
        }
        nodes_of_passage.push_back(*iter);
    }
    nodes_of_passages->push_back(nodes_of_passage);
}

bool is_reachable_from(const TopoNode* node,
                       const std::vector<const TopoNode*>& to_nodes) {
    for (const auto& to_node : to_nodes) {
        auto edge = to_node->get_in_edge_from(node);
        if (edge != nullptr) {
            if (edge->type() == TET_LEFT || edge->type() == TET_RIGHT) {
                return true;
            }
        }
    }
    return false;
}

bool is_reachable_to(const TopoNode* node,
                     const std::vector<const TopoNode*>& from_nodes) {
    for (const auto& from_node : from_nodes) {
        auto edge = from_node->get_out_edge_to(node);
        if (edge != nullptr) {
            if (edge->type() == TET_LEFT || edge->type() == TET_RIGHT) {
                return true;
            }
        }
    }
    return false;
}

void extend_backward(const std::vector<const TopoNode*>& nodes_of_prev_passage,
                     const std::unordered_set<const TopoNode*>& black_list,
                     std::vector<const TopoNode*>* const nodes_of_curr_passage) {
    bool is_reachable = true;
    while (is_reachable) {
        is_reachable = false;
        for (const auto& edge : nodes_of_curr_passage->front()->in_from_pre_edge()) {
            const auto& pred_node = edge->from_node();
            // if pred node is not in the same road
            if (pred_node->road_id() != nodes_of_curr_passage->front()->road_id()) {
                continue;
            }
            // if pred node has been inserted
            if (find(nodes_of_curr_passage->begin(),
                     nodes_of_curr_passage->end(),
                     pred_node) != nodes_of_curr_passage->end()) {
                continue;
            }
            is_reachable = is_reachable_to(pred_node, nodes_of_prev_passage);
            // if pred node is reachable from prev passage and not in black list
            if (is_reachable && black_list.find(pred_node) == black_list.end()) {
                nodes_of_curr_passage->insert(nodes_of_curr_passage->begin(), pred_node);
                break;
            // still need to explore other pred nodes
            } else {
                continue;
            }
        }
    }
}

void extend_forward(const std::vector<const TopoNode*>& nodes_of_next_passage,
                    const std::unordered_set<const TopoNode*>& black_list,
                    std::vector<const TopoNode*>* const nodes_of_curr_passage) {
    bool is_reachable = true;
    while (is_reachable) {
        is_reachable = false;
        for (const auto& edge : nodes_of_curr_passage->back()->out_to_suc_edge()) {
            const auto& succ_node = edge->to_node();
            // if succ node is not in the same road
            if (succ_node->road_id() != nodes_of_curr_passage->back()->road_id()) {
                continue;
            }
            // if succ node has been inserted
            if (find(nodes_of_curr_passage->begin(),
                        nodes_of_curr_passage->end(),
                        succ_node) != nodes_of_curr_passage->end()) {
                continue;
            }
            is_reachable = is_reachable_from(succ_node, nodes_of_next_passage);
            // if next passage is reachable from succ node and not in black list
            if (is_reachable && black_list.find(succ_node) == black_list.end()) {
                nodes_of_curr_passage->push_back(succ_node);
                break;
            // still need to explore other succ nodes
            } else {
                continue;
            }
        }
    }
}

void extend_passages(const std::unordered_set<const TopoNode*>& black_list,
                     std::vector<std::vector<const TopoNode*> >* const nodes_of_passages) {
    for (size_t i = 0; i < nodes_of_passages->size(); ++i) {
        if (i < nodes_of_passages->size() - 1) {
            extend_forward(nodes_of_passages->at(i + 1),
                           black_list,
                           &(nodes_of_passages->at(i)));
        }
        if (i > 0) {
            extend_backward(nodes_of_passages->at(i - 1),
                            black_list,
                            &(nodes_of_passages->at(i)));
        }
    }
}

void passage_lane_ids_to_passage_region(const std::vector<const TopoNode*>& nodes,
                                        const NodeSRangeManager& range_manager,
                                        RoutingResult_PassageRegion* const region) {
    for (const auto& node : nodes) {
        RoutingResult_LaneSegment* seg = region->add_segment();
        seg->set_id(node->lane_id());
        NodeSRange range = range_manager.get_node_range(node);
        seg->set_start_s(range.start_s());
        seg->set_end_s(range.end_s());
    }
}

double calculate_distance(const std::vector<const TopoNode*>& nodes,
                          const NodeSRangeManager& range_manager) {
    NodeSRange range = range_manager.get_node_range(nodes.at(0));
    double distance = range.end_s() - range.start_s();
    for (auto iter = nodes.begin() + 1; iter != nodes.end(); ++iter) {
        auto edge = (*(iter - 1))->get_out_edge_to(*iter);
        if (edge->type() != TET_FORWARD) {
            continue;
        }
        range = range_manager.get_node_range(*iter);
        distance += range.end_s() - range.start_s();
    }
    return distance;
}

} // namespace

// new request to new response
bool RouterResultGenerator::generate_passage_region(
                                        const std::string& map_version,
                                        const ::adu::common::router::RoutingRequest& request,
                                        const std::vector<const TopoNode*>& nodes,
                                        const std::unordered_set<const TopoNode*>& black_list,
                                        const NodeSRangeManager& range_manager,
                                        ::adu::common::router::RoutingResult* result) const {
    result->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    result->mutable_header()->set_module_name(FLAGS_node_name);
    result->mutable_header()->set_sequence_num(1);

    generate_passage_region(nodes, black_list, range_manager, result);

    result->set_map_version(map_version);
    result->mutable_measurement()->set_distance(calculate_distance(nodes, range_manager));
    result->mutable_routing_request()->CopyFrom(request);
    return true;
}

// use internal generate result
void RouterResultGenerator::generate_passage_region(const std::vector<const TopoNode*>& nodes,
                                        const std::unordered_set<const TopoNode*>& black_list,
                                        const NodeSRangeManager& range_manager,
                                        ::adu::common::router::RoutingResult* result) const {
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
            std::vector<RoutingResult_LaneChangeInfo::Type> lane_change_types;
            extract_basic_passages(nodes_of_way,
                                   &nodes_of_basic_passages,
                                   &lane_change_types);
            for (const auto& nodes_of_passage : nodes_of_basic_passages) {
                ROS_INFO("New passage!!!");
                for (const auto& node : nodes_of_passage) {
                    ROS_INFO("lane id: %s", node->lane_id().c_str());
                }
            }
            extend_passages(black_list, &nodes_of_basic_passages);
            ROS_INFO("------After extend--------");
            for (const auto& nodes_of_passage : nodes_of_basic_passages) {
                ROS_INFO("New passage!!!");
                for (const auto& node : nodes_of_passage) {
                    ROS_INFO("lane id: %s", node->lane_id().c_str());
                }
            }

            RoutingResult_Road road;
            road.set_id("r" + boost::lexical_cast<std::string>(num_of_roads));
            auto node = nodes_of_basic_passages.front().front();
            road.mutable_in_lane()->set_id(node->lane_id());
            road.mutable_in_lane()->set_s(range_manager.get_node_start_s(node));
            node = nodes_of_basic_passages.back().back();
            road.mutable_out_lane()->set_id(node->lane_id());
            road.mutable_out_lane()->set_s(range_manager.get_node_end_s(node));
            for (const auto& nodes_of_passage : nodes_of_basic_passages) {
                passage_lane_ids_to_passage_region(nodes_of_passage,
                                                   range_manager,
                                                   road.add_passage_region());
                // TODO: set trajectory of region
            }
            for (size_t i = 0; i < lane_change_types.size(); ++i) {
                RoutingResult_LaneChangeInfo* lc_info = road.add_lane_change_info();
                lc_info->set_type(lane_change_types.at(i));
                lc_info->set_start_passage_region_index(i);
                lc_info->set_end_passage_region_index(i + 1);
            }
            result->add_route()->mutable_road_info()->CopyFrom(road);
            num_of_roads++;
        } else {
            RoutingResult_Junction junction;
            junction.set_id("j" + boost::lexical_cast<std::string>(num_of_junctions));
            junction.set_in_road_id("r" + boost::lexical_cast<std::string>(num_of_roads - 1));
            junction.set_out_road_id("r" + boost::lexical_cast<std::string>(num_of_roads));
            RoutingResult_PassageRegion region;
            for (const auto& node : nodes_of_way) {
                RoutingResult_LaneSegment* seg = region.add_segment();
                seg->set_id(node->lane_id());
                NodeSRange range = range_manager.get_node_range(node);
                seg->set_start_s(range.start_s());
                seg->set_end_s(range.end_s());
            }
            // TODO: set trajectory of region
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

} // namespace routing
} // namespace adu

