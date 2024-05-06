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

#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/routing/graph/node_with_range.h"

namespace apollo {
namespace routing {

class SubTopoGraph {
 public:
  SubTopoGraph(const std::unordered_map<const TopoNode*,
                                        std::vector<NodeSRange>>& black_map);
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
  void GetSubInEdgesIntoSubGraph(
      const TopoEdge* edge,
      std::unordered_set<const TopoEdge*>* const sub_edges) const;

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
  void GetSubOutEdgesIntoSubGraph(
      const TopoEdge* edge,
      std::unordered_set<const TopoEdge*>* const sub_edges) const;

  const TopoNode* GetSubNodeWithS(const TopoNode* topo_node, double s) const;

 private:
  void InitSubNodeByValidRange(const TopoNode* topo_node,
                               const std::vector<NodeSRange>& valid_range);
  void InitSubEdge(const TopoNode* topo_node);

  void InitInSubNodeSubEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);
  void InitOutSubNodeSubEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);

  bool GetSubNodes(const TopoNode* node,
                   std::unordered_set<TopoNode*>* const sub_nodes) const;

  void AddPotentialEdge(const TopoNode* topo_node);
  void AddPotentialInEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);
  void AddPotentialOutEdge(
      TopoNode* const sub_node,
      const std::unordered_set<const TopoEdge*> origin_edge);

 private:
  std::vector<std::shared_ptr<TopoNode>> topo_nodes_;
  std::vector<std::shared_ptr<TopoEdge>> topo_edges_;
  std::unordered_map<const TopoNode*, std::vector<NodeWithRange>>
      sub_node_range_sorted_map_;
  std::unordered_map<const TopoNode*, std::unordered_set<TopoNode*>>
      sub_node_map_;
};

}  // namespace routing
}  // namespace apollo
