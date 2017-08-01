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

#ifndef BAIDU_ADU_ROUTING_GRAPH_TOPO_GRAPH_H
#define BAIDU_ADU_ROUTING_GRAPH_TOPO_GRAPH_H

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "topo_graph.pb.h"

namespace adu {
namespace routing {

class TopoNode;
class TopoEdge;

class TopoGraph {
 public:
  TopoGraph() = default;
  ~TopoGraph() = default;

  bool load_graph(const std::string& filename);

  const std::string& map_version() const;
  const std::string& map_district() const;
  const ::adu::routing::TopoNode* get_node(const std::string& id) const;
  void get_nodes_by_road_id(
      const std::string& road_id,
      std::unordered_set<const TopoNode*>* const node_in_road) const;

 private:
  void clear();
  bool load_nodes(const ::adu::routing::common::Graph& graph);
  bool load_edges(const ::adu::routing::common::Graph& graph);

 private:
  std::string _map_version;
  std::string _map_district;
  std::vector<std::shared_ptr<TopoNode> > _topo_nodes;
  std::vector<std::shared_ptr<TopoEdge> > _topo_edges;
  std::unordered_map<std::string, int> _node_index_map;
  std::unordered_map<std::string, std::unordered_set<const TopoNode*> >
      _road_node_map;
};

}  // namespace routing
}  // namespace adu

#endif  // BAIDU_ADU_ROUTING_GRAPH_TOPO_GRAPH_H
