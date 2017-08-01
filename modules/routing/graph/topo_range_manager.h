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

#ifndef BAIDU_ADU_ROUTING_GRAPH_TOPO_RANGE_MANAGER_H
#define BAIDU_ADU_ROUTING_GRAPH_TOPO_RANGE_MANAGER_H

#include <unordered_map>
#include <unordered_set>

#include "graph/topo_range.h"

namespace adu {
namespace routing {

class TopoNode;

class NodeSRangeManager {
public:
    NodeSRangeManager() = default;
    ~NodeSRangeManager() = default;

    void clear();
    void init_node_range(double start_node_s,
                         double end_node_s,
                         const TopoNode* start_node,
                         const TopoNode* end_node);

    NodeSRange get_node_range(const TopoNode* topo_node) const;
    double get_node_start_s(const TopoNode* topo_node) const;
    double get_node_end_s(const TopoNode* topo_node) const;

private:
    void init_in_end(const TopoNode* cur_node, double end_s);
    void init_out_start(const TopoNode* cur_node, double start_s);
    void init_in_neighbor(const TopoNode* cur_node, double start_s, double end_s);
    void init_out_neighbor(const TopoNode* cur_node, double start_s, double end_s);

private:
    std::unordered_map<const TopoNode*, NodeSRange> _range_map;
    std::unordered_set<const TopoNode*> _is_start_inited;
    std::unordered_set<const TopoNode*> _is_end_inited;
};

} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTING_GRAPH_TOPO_RANGE_MANAGER_H

