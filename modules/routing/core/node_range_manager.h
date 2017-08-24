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

#ifndef MODULES_ROUTING_CORE_NODE_RANGE_MANAGER_H_
#define MODULES_ROUTING_CORE_NODE_RANGE_MANAGER_H_

#include <unordered_map>

namespace apollo {
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

  void InitNodeRange(double start_node_s, double end_node_s,
                     const TopoNode* start_node, const TopoNode* end_node);

  NodeRange GetNodeRange(const TopoNode* topo_node) const;
  double GetNodeStartS(const TopoNode* topo_node) const;
  double GetNodeEndS(const TopoNode* topo_node) const;
  void SetNodeS(const TopoNode* topo_node,
                double node_start_s, double node_end_s);

 private:
  void InitInNeighbor(const TopoNode* cur_node, double start_s, double end_s);
  void InitOutNeighbor(const TopoNode* cur_node, double start_s, double end_s);

 private:
  std::unordered_map<const TopoNode*, NodeRange> range_map_;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_CORE_NODE_RANGE_MANAGER_H_
