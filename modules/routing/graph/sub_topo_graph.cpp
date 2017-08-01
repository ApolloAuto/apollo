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

#include "graph/sub_topo_graph.h"

#include <cmath>

#include "graph/topo_edge.h"
#include "graph/topo_node.h"

#include "glog/logging.h"

namespace adu {
namespace routing {

namespace {

void merge_block_range(const TopoNode* topo_node,
                       const std::vector<NodeSRange>& origin_range,
                       std::vector<NodeSRange>* block_range) {
    std::vector<NodeSRange> sorted_origin_range;
    sorted_origin_range.insert(sorted_origin_range.end(),
                               origin_range.begin(),
                               origin_range.end());
    sort(sorted_origin_range.begin(), sorted_origin_range.end());
    int cur_index = 0;
    int total_size = sorted_origin_range.size();
    while (cur_index < total_size) {
        NodeSRange range(sorted_origin_range[cur_index]);
        ++cur_index;
        while (cur_index < total_size &&
               range.merge_range_overlap(sorted_origin_range[cur_index])) {
            ++cur_index;
        }
        if (range.end_s() < topo_node->start_s() || range.start_s() > topo_node->end_s()) {
            continue;
        }
        range.set_start_s(std::max(topo_node->start_s(), range.start_s()));
        range.set_end_s(std::min(topo_node->end_s(), range.end_s()));
        block_range->push_back(std::move(range));
    }
}

void get_sorted_valid_range(const TopoNode* topo_node,
                            const std::vector<NodeSRange>& origin_range,
                            std::vector<NodeSRange>* valid_range) {
    std::vector<NodeSRange> block_range;
    merge_block_range(topo_node, origin_range, &block_range);
    double start_s = topo_node->start_s();
    double end_s = topo_node->end_s();
    std::vector<double> all_value;
    all_value.push_back(start_s);
    for (const auto& range : block_range) {
        all_value.push_back(range.start_s());
        all_value.push_back(range.end_s());
    }
    all_value.push_back(end_s);
    for (size_t i = 0; i < all_value.size(); i += 2) {
        if (!NodeSRange::is_enough_for_change_lane(all_value[i], all_value[i + 1])) {
            continue;
        }
        NodeSRange new_range(all_value[i], all_value[i + 1]);
        valid_range->push_back(std::move(new_range));
    }
}

} // namespace

SubTopoGraph::SubTopoGraph(const std::vector<NodeWithRange>& black_list) {
    std::unordered_map<const TopoNode*, std::vector<NodeSRange> > black_map;
    for (const auto& node_in_bl : black_list) {
        black_map[node_in_bl.topo_node()].push_back(node_in_bl);
    }
    std::vector<NodeSRange> valid_range;
    for (auto& map_iter : black_map) {
        valid_range.clear();
        get_sorted_valid_range(map_iter.first, map_iter.second, &valid_range);
        init_sub_node_by_valid_range(map_iter.first, valid_range);
    }

    for (const auto& map_iter : black_map) {
        init_sub_edge(map_iter.first);
    }
}

SubTopoGraph::~SubTopoGraph() { }

void SubTopoGraph::get_sub_edges_into_sub_graph(const TopoEdge* edge,
                                 std::unordered_set<const TopoEdge*>* const sub_edges) const {
    const auto* from_node = edge->from_node();
    const auto* to_node = edge->to_node();
    std::unordered_set<TopoNode*> sub_nodes;
    if (from_node->is_sub_node() || to_node->is_sub_node() || !get_sub_nodes(to_node, &sub_nodes)) {
        sub_edges->insert(edge);
        return;
    }
    for (const auto* sub_node : sub_nodes) {
        for (const auto* in_edge : sub_node->in_from_all_edge()) {
            if (in_edge->from_node() == from_node) {
                sub_edges->insert(in_edge);
            }
        }
    }
}

const TopoNode* SubTopoGraph::get_sub_node_with_s(const TopoNode* topo_node, double s) const {
    const auto& map_iter = _sub_node_range_sorted_map.find(topo_node);
    if (map_iter == _sub_node_range_sorted_map.end()) {
        return topo_node;
    }
    const auto& sorted_vec = map_iter->second;
    // sorted vec can't be empty!
    int index = binary_search_for_start_s(sorted_vec, s);
    if (index < 0) {
        return nullptr;
    }
    return sorted_vec[index].topo_node();
}

void SubTopoGraph::init_sub_node_by_valid_range(const TopoNode* topo_node,
                                                const std::vector<NodeSRange>& valid_range) {
    // Attention: no matter topo node has valid_range or not,
    // create map value first;
    auto& sub_node_vec = _sub_node_range_sorted_map[topo_node];
    auto& sub_node_set = _sub_node_map[topo_node];
    for (const auto& range : valid_range) {
        std::shared_ptr<TopoNode> sub_topo_node_ptr;
        sub_topo_node_ptr.reset(new TopoNode(topo_node, range));
        NodeWithRange node_with_range(range, sub_topo_node_ptr.get());
        sub_node_vec.push_back(std::move(node_with_range));
        sub_node_set.insert(sub_topo_node_ptr.get());
        _topo_nodes.push_back(std::move(sub_topo_node_ptr));
    }
}

void SubTopoGraph::init_sub_edge(const TopoNode* topo_node) {
    std::unordered_set<TopoNode*> sub_nodes;
    if (!get_sub_nodes(topo_node, &sub_nodes)) {
        return;
    }
    double start_s = topo_node->start_s();
    double end_s = topo_node->end_s();
    const double MIN_DIFF_LENGTH = 0.1e-6;

    for (auto* sub_node : sub_nodes) {
        init_in_sub_node_sub_edge(sub_node, topo_node->in_from_left_or_right_edge());
        init_out_sub_node_sub_edge(sub_node, topo_node->out_to_left_or_right_edge());
        if (std::fabs(sub_node->start_s() - start_s) < MIN_DIFF_LENGTH) {
            init_in_sub_node_sub_edge(sub_node, topo_node->in_from_pre_edge());
        }
        if (std::fabs(end_s - sub_node->end_s()) < MIN_DIFF_LENGTH) {
            init_out_sub_node_sub_edge(sub_node, topo_node->out_to_suc_edge());
        }
    }
}

void SubTopoGraph::init_in_sub_edge(const TopoNode* sub_node,
                                    const std::unordered_set<const TopoEdge*> origin_edge) {
    std::unordered_set<TopoNode*> other_sub_nodes;
    for (const auto* in_edge : origin_edge) {
        if (get_sub_nodes(in_edge->from_node(), &other_sub_nodes)) {
            for (auto* sub_from_node : other_sub_nodes) {
                if (!sub_from_node->is_overlap_enough(sub_node, in_edge)) {
                    continue;
                }
                std::shared_ptr<TopoEdge> topo_edge_ptr;
                topo_edge_ptr.reset(new TopoEdge(in_edge->edge(), sub_from_node, sub_node));
                sub_from_node->add_out_edge(topo_edge_ptr.get());
                _topo_edges.push_back(std::move(topo_edge_ptr));
            }
        }
    }
}

void SubTopoGraph::init_in_sub_node_sub_edge(TopoNode* const sub_node,
                                const std::unordered_set<const TopoEdge*> origin_edge) {
    std::unordered_set<TopoNode*> other_sub_nodes;
    for (const auto* in_edge : origin_edge) {
        if (get_sub_nodes(in_edge->from_node(), &other_sub_nodes)) {
            for (auto* sub_from_node : other_sub_nodes) {
                if (!sub_from_node->is_overlap_enough(sub_node, in_edge)) {
                    continue;
                }
                std::shared_ptr<TopoEdge> topo_edge_ptr;
                topo_edge_ptr.reset(new TopoEdge(in_edge->edge(), sub_from_node, sub_node));
                sub_node->add_in_edge(topo_edge_ptr.get());
                sub_from_node->add_out_edge(topo_edge_ptr.get());
                _topo_edges.push_back(std::move(topo_edge_ptr));
            }
        } else {
            std::shared_ptr<TopoEdge> topo_edge_ptr;
            topo_edge_ptr.reset(new TopoEdge(in_edge->edge(), in_edge->from_node(), sub_node));
            sub_node->add_in_edge(topo_edge_ptr.get());
            _topo_edges.push_back(std::move(topo_edge_ptr));
        }
    }
}

void SubTopoGraph::init_out_sub_edge(const TopoNode* sub_node,
                                     const std::unordered_set<const TopoEdge*> origin_edge) {
    std::unordered_set<TopoNode*> other_sub_nodes;
    for (const auto* out_edge : origin_edge) {
        if (get_sub_nodes(out_edge->to_node(), &other_sub_nodes)) {
            for (auto* sub_to_node : other_sub_nodes) {
                if (!sub_node->is_overlap_enough(sub_to_node, out_edge)) {
                    continue;
                }
                std::shared_ptr<TopoEdge> topo_edge_ptr;
                topo_edge_ptr.reset(new TopoEdge(out_edge->edge(), sub_node, sub_to_node));
                sub_to_node->add_out_edge(topo_edge_ptr.get());
                _topo_edges.push_back(std::move(topo_edge_ptr));
            }
        }
    }
}

void SubTopoGraph::init_out_sub_node_sub_edge(TopoNode* const sub_node,
                                const std::unordered_set<const TopoEdge*> origin_edge) {
    std::unordered_set<TopoNode*> other_sub_nodes;
    for (const auto* out_edge : origin_edge) {
        if (get_sub_nodes(out_edge->to_node(), &other_sub_nodes)) {
            for (auto* sub_to_node : other_sub_nodes) {
                if (!sub_node->is_overlap_enough(sub_to_node, out_edge)) {
                    continue;
                }
                std::shared_ptr<TopoEdge> topo_edge_ptr;
                topo_edge_ptr.reset(new TopoEdge(out_edge->edge(), sub_node, sub_to_node));
                sub_to_node->add_out_edge(topo_edge_ptr.get());
                sub_node->add_out_edge(topo_edge_ptr.get());
                _topo_edges.push_back(std::move(topo_edge_ptr));
            }
        } else {
            std::shared_ptr<TopoEdge> topo_edge_ptr;
            topo_edge_ptr.reset(new TopoEdge(out_edge->edge(), sub_node, out_edge->to_node()));
            sub_node->add_out_edge(topo_edge_ptr.get());
            _topo_edges.push_back(std::move(topo_edge_ptr));
        }
    }
}

bool SubTopoGraph::get_sub_nodes(const TopoNode* node,
                                 std::unordered_set<TopoNode*>* const sub_nodes) const {
    const auto& iter = _sub_node_map.find(node);
    if (iter == _sub_node_map.end()) {
        return false;
    }
    sub_nodes->clear();
    sub_nodes->insert(iter->second.begin(), iter->second.end());
    return true;
}

} // namespace routing
} // namespace adu

