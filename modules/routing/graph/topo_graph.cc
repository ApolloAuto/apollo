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

#include <utility>

namespace apollo {
namespace routing {

void TopoGraph::Clear() {
  topo_nodes_.clear();
  topo_edges_.clear();
  node_index_map_.clear();
}

bool TopoGraph::LoadNodes(const Graph& graph) {
  // 拓扑图中没有节点
  if (graph.node().empty()) {
    AERROR << "No nodes found in topology graph.";
    return false;
  }

  // 遍历graph中所有节点：一个节点代表一条lane
  for (const auto& node : graph.node()) {
    // 使用节点的车道ID (lane_id()) 作为键，映射到当前已加载的节点数量（即新节点的索引）
    // 映射便于根据车道ID快速定位节点在 topo_nodes_ 中的位置
    node_index_map_[node.lane_id()] = static_cast<int>(topo_nodes_.size());

    // 创建一个新的 TopoNode 对象，使用当前遍历的 node protobuf 数据初始化
    std::shared_ptr<TopoNode> topo_node;
    topo_node.reset(new TopoNode(node));

    // 将该 TopoNode 的裸指针插入以道路ID (road_id()) 为键的 road_node_map_ 容器中
    // 该映射用于快速根据道路ID查询所有属于该道路的拓扑节点
    road_node_map_[node.road_id()].insert(topo_node.get());

    // 把智能指针 topo_node 添加到成员变量 topo_nodes_ 中，topo_nodes_ 是存储所有拓扑节点的容器
    topo_nodes_.push_back(std::move(topo_node));
  }
  return true;
}

// Need to execute load_nodes() firstly
bool TopoGraph::LoadEdges(const Graph& graph) {
// 拓扑图中没有边，不过没关系
  if (graph.edge().empty()) {
    AINFO << "0 edges found in topology graph, but it's fine";
    return true;
  }

  // 遍历 graph 中每一条边
  for (const auto& edge : graph.edge()) {
    // 获取该边的起点车道ID（from_lane_id）和终点车道ID（to_lane_id）
    const std::string& from_lane_id = edge.from_lane_id();
    const std::string& to_lane_id = edge.to_lane_id();
    // 检查这两个车道ID是否都出现在已加载的节点映射表中
    if (node_index_map_.count(from_lane_id) != 1 ||
        node_index_map_.count(to_lane_id) != 1) {
      return false;
    }

    // 根据索引获取 from_node 和 to_node 的原始指针
    // 这两个节点对象构成边的两端
    std::shared_ptr<TopoEdge> topo_edge;
    TopoNode* from_node = topo_nodes_[node_index_map_[from_lane_id]].get();
    TopoNode* to_node = topo_nodes_[node_index_map_[to_lane_id]].get();

    // 创建一个新的 TopoEdge 对象，使用当前 protobuf 中的 edge 数据以及起点和终点节点初始化
    topo_edge.reset(new TopoEdge(edge, from_node, to_node));

    // 将该边加入到 from_node 的“出边”列表（即这个节点出去的边）
    from_node->AddOutEdge(topo_edge.get());
    // 同时加入到 to_node 的“入边”列表（即这个节点进来的边）
    to_node->AddInEdge(topo_edge.get());
    // 把该边对象加入 topo_edges_ 容器中，保存所有边的集合
    topo_edges_.push_back(std::move(topo_edge));
  }
  return true;
}

bool TopoGraph::LoadGraph(const Graph& graph) {
  // 清空当前拓扑图数据，重置成员变量
  Clear();

  map_version_ = graph.hdmap_version();
  map_district_ = graph.hdmap_district();

  // 加载拓扑节点（路网中的关键点）
  if (!LoadNodes(graph)) {
    AERROR << "Failed to load nodes from topology graph.";
    return false;
  }

  // 加载拓扑边（连接节点的道路段）
  if (!LoadEdges(graph)) {
    AERROR << "Failed to load edges from topology graph.";
    return false;
  }
  AINFO << "Load Topo data successful.";
  return true;
}

const std::string& TopoGraph::MapVersion() const { return map_version_; }

const std::string& TopoGraph::MapDistrict() const { return map_district_; }

/// @brief 
/// @param id 一个车道 ID（string），比如 lane_id
/// @return 
const TopoNode* TopoGraph::GetNode(const std::string& id) const {
  // node_index_map_： std::unordered_map<std::string, int>
  // 从 lane_id 映射到索引，快速找到这个 ID 在 topo_nodes_ 中的位置
  const auto& iter = node_index_map_.find(id);
  if (iter == node_index_map_.end()) {
    return nullptr;
  }
  // topo_nodes_： std::vector<std::unique_ptr<TopoNode>> 保存了所有的 TopoNode 对象。每个 TopoNode 用智能指针管理
  return topo_nodes_[iter->second].get();
}

/*
node_index_map_["lane_123"] = 5;
topo_nodes_[5] = std::make_unique<TopoNode>("lane_123", ...);

const TopoNode* node = topo_graph.GetNode("lane_123");

topo_nodes_[5].get();
*/

void TopoGraph::GetNodesByRoadId(
    const std::string& road_id,
    std::unordered_set<const TopoNode*>* const node_in_road) const {
  const auto& iter = road_node_map_.find(road_id);
  if (iter != road_node_map_.end()) {
    node_in_road->insert(iter->second.begin(), iter->second.end());
  }
}

}  // namespace routing
}  // namespace apollo
