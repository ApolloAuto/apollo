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

#ifndef BAIDU_ADU_ROUTING_GRAPH_SUB_TOPO_GRAPH_H
#define BAIDU_ADU_ROUTING_GRAPH_SUB_TOPO_GRAPH_H

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <queue>

#include "graph/topo_graph.h"
#include "graph/topo_range.h"

namespace adu {
namespace routing {

class TopoNode;
class TopoEdge;
class TopoGraph;

class SubTopoGraph {
public:
    explicit SubTopoGraph(const std::vector<NodeWithRange>& black_list);
    ~SubTopoGraph();

    // edge: A -> B         not sub edge
    // 1. A has no sub node, B has no sub node
    //      return origin edge A -> B
    // 2. A has no sub node, B has sub node
    //      return all edge A -> B'
    //    if B is black lane and no valid edge,
    //      return empty set
    // 3. A has sub node, B has sub node
    //      return empty set
    // 4. A has sub node, B has no sub node
    //      return empty set
    // edge: A -> B is sub edge
    // 1. return empty set
    void get_sub_edges_into_sub_graph(const TopoEdge* edge,
                       std::unordered_set<const TopoEdge*>* const sub_edges) const;

    const TopoNode* get_sub_node_with_s(const TopoNode* topo_node, double s) const;

private:
    void init_sub_node_by_valid_range(const TopoNode* topo_node,
                                      const std::vector<NodeSRange>& valid_range);
    void init_sub_edge(const TopoNode* topo_node);

    void init_in_sub_edge(const TopoNode* sub_node,
                          const std::unordered_set<const TopoEdge*> origin_edge);
    void init_in_sub_node_sub_edge(TopoNode* const sub_node,
                                   const std::unordered_set<const TopoEdge*> origin_edge);
    void init_out_sub_edge(const TopoNode* sub_node,
                           const std::unordered_set<const TopoEdge*> origin_edge);
    void init_out_sub_node_sub_edge(TopoNode* const sub_node,
                                    const std::unordered_set<const TopoEdge*> origin_edge);

    bool get_sub_nodes(const TopoNode* node, std::unordered_set<TopoNode*>* const sub_nodes) const;

private:
    std::vector<std::shared_ptr<TopoNode> > _topo_nodes;
    std::vector<std::shared_ptr<TopoEdge> > _topo_edges;
    std::unordered_map<const TopoNode*, std::vector<NodeWithRange> > _sub_node_range_sorted_map;
    std::unordered_map<const TopoNode*, std::unordered_set<TopoNode*> > _sub_node_map;
};

} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTING_GRAPH_SUB_TOPO_GRAPH_H

