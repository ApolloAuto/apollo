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

#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/common/utils.h"
#include "modules/routing/graph/topo_node.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace routing {

void TopoGraph::clear() {
  _topo_nodes.clear();
  _topo_edges.clear();
  _node_index_map.clear();
}

bool TopoGraph::load_nodes(const ::apollo::routing::Graph& graph) {
  if (graph.node_size() == 0) {
    AERROR << "No nodes found in topology graph.";
    return false;
  }
  for (const auto& node : graph.node()) {
    _node_index_map[node.lane_id()] = _topo_nodes.size();
    std::shared_ptr<TopoNode> topo_node;
    topo_node.reset(new TopoNode(node));
    _road_node_map[node.road_id()].insert(topo_node.get());
    _topo_nodes.push_back(std::move(topo_node));
  }
  return true;
}

// Need to execute load_nodes() firstly
bool TopoGraph::load_edges(const ::apollo::routing::Graph& graph) {
  if (graph.edge_size() == 0) {
    AINFO << "0 edges found in topology graph, but it's fine";
    return true;
  }
  for (const auto& edge : graph.edge()) {
    const std::string& from_lane_id = edge.from_lane_id();
    const std::string& to_lane_id = edge.to_lane_id();
    if (_node_index_map.count(from_lane_id) != 1 ||
        _node_index_map.count(to_lane_id) != 1) {
      return false;
    }
    std::shared_ptr<TopoEdge> topo_edge;
    TopoNode* from_node = _topo_nodes[_node_index_map[from_lane_id]].get();
    TopoNode* to_node = _topo_nodes[_node_index_map[to_lane_id]].get();
    topo_edge.reset(new TopoEdge(edge, from_node, to_node));
    from_node->add_out_edge(topo_edge.get());
    to_node->add_in_edge(topo_edge.get());
    _topo_edges.push_back(std::move(topo_edge));
  }
  return true;
}

bool TopoGraph::load_graph(const std::string& file_path) {
  clear();

  ::apollo::routing::Graph graph;
  if (!::apollo::common::util::GetProtoFromFile(
          file_path, &graph)) {
    AERROR << "Failed to read topology graph from data.";
    AERROR << "File Path: " << file_path;
    return false;
  }

  _map_version = graph.hdmap_version();
  _map_district = graph.hdmap_district();

  if (!load_nodes(graph)) {
    AERROR << "Failed to load nodes from topology graph.";
    return false;
  }
  if (!load_edges(graph)) {
    AERROR << "Failed to load edges from topology graph.";
    return false;
  }
  AERROR << "Load Topo data succesful.";
  return true;
}

const std::string& TopoGraph::map_version() const { return _map_version; }

const std::string& TopoGraph::map_district() const { return _map_district; }

const TopoNode* TopoGraph::get_node(const std::string& id) const {
  const auto& iter = _node_index_map.find(id);
  if (iter == _node_index_map.end()) {
    return nullptr;
  }
  return _topo_nodes[iter->second].get();
}

void TopoGraph::get_nodes_by_road_id(
    const std::string& road_id,
    std::unordered_set<const TopoNode*>* const node_in_road) const {
  const auto& iter = _road_node_map.find(road_id);
  if (iter != _road_node_map.end()) {
    node_in_road->insert(iter->second.begin(), iter->second.end());
  }
}

}  // namespace routing
}  // namespace apollo
