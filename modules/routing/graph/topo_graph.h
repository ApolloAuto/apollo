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

#ifndef MODULES_ROUTING_GRAPH_TOPO_GRAPH_H
#define MODULES_ROUTING_GRAPH_TOPO_GRAPH_H

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/log.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
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
  const TopoNode* get_node(const std::string& id) const;
  void get_nodes_by_road_id(
      const std::string& road_id,
      std::unordered_set<const TopoNode*>* const node_in_road) const;

 private:
  void clear();
  bool load_nodes(const Graph& graph);
  bool load_edges(const Graph& graph);

 private:
  std::string _map_version;
  std::string _map_district;
  std::vector<std::shared_ptr<TopoNode> > topo_nodes_;
  std::vector<std::shared_ptr<TopoEdge> > topo_edges_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::unordered_set<const TopoNode*> >
      road_node_map_;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_GRAPH_TOPO_GRAPH_H
