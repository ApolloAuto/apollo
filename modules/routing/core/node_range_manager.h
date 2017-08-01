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

#ifndef BAIDU_ADU_ROUTING_CORE_NODE_RANGE_MANAGER_H
#define BAIDU_ADU_ROUTING_CORE_NODE_RANGE_MANAGER_H

#include <unordered_map>

namespace adu {
namespace routing {

struct NodeRange {
  double start_s;
  double end_s;
  NodeRange() : start_s(0.0), end_s(0.0) {}
  NodeRange(double s1, double s2) : start_s(s1), end_s(s2) {}
};

class TopoNode;

class NodeRangeManager {
 public:
  NodeRangeManager() = default;
  ~NodeRangeManager() = default;

  void init_node_range(double start_node_s, double end_node_s,
                       const TopoNode* start_node, const TopoNode* end_node);

  NodeRange get_node_range(const TopoNode* topo_node) const;
  double get_node_start_s(const TopoNode* topo_node) const;
  double get_node_end_s(const TopoNode* topo_node) const;

 private:
  void init_in_neighbor(const TopoNode* cur_node, double start_s, double end_s);
  void init_out_neighbor(const TopoNode* cur_node, double start_s,
                         double end_s);

 private:
  std::unordered_map<const TopoNode*, NodeRange> _range_map;
};

}  // namespace routing
}  // namespace adu

#endif  // BAIDU_ADU_ROUTING_CORE_NODE_RANGE_MANAGER_H
