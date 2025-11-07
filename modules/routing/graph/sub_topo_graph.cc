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

#include "modules/routing/graph/sub_topo_graph.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/routing/graph/range_utils.h"

namespace apollo {
namespace routing {

namespace {

const double MIN_DIFF_LENGTH = 0.1e-6;             // in meters
const double MIN_INTERNAL_FOR_NODE = 0.01;         // in meters
const double MIN_POTENTIAL_LANE_CHANGE_LEN = 3.0;  // in meters

bool IsCloseEnough(double s1, double s2) {
  return std::fabs(s1 - s2) < MIN_DIFF_LENGTH;
}

/// @brief 将输入的原始区间集合（origin_range）按顺序合并重叠或相邻的区间，并剪裁到车道有效区间内，最后输出到 block_range
/// @param topo_node 
/// @param origin_range 
/// @param block_range 
void MergeBlockRange(const TopoNode* topo_node,
                     const std::vector<NodeSRange>& origin_range,
                     std::vector<NodeSRange>* block_range) {
  // 复制输入的原始区间 origin_range 到本地 sorted_origin_range
  std::vector<NodeSRange> sorted_origin_range;
  sorted_origin_range.insert(sorted_origin_range.end(), origin_range.begin(),
                             origin_range.end());
  // 对 sorted_origin_range 进行排序（默认排序按 NodeSRange 的起始位置排序，前提是 NodeSRange 有对应的 operator< 实现）
  sort(sorted_origin_range.begin(), sorted_origin_range.end());

  // 初始化循环索引，准备遍历排序后的区间列表
  int cur_index = 0;
  int total_size = static_cast<int>(sorted_origin_range.size());

  // 取当前索引区间作为合并区间的初始值
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;

    // 内层循环试图把所有与当前 range 重叠或相邻的区间合并进 range 中
    // MergeRangeOverlap：如果参数区间和 range 有重叠或紧邻，合并它们，扩展 range，返回 true
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {
      ++cur_index;
    }

    // 判断合并后的区间是否完全在车道有效区间外（车道起点到终点）
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }

    // 将区间裁剪到车道有效范围内（保证区间起点不小于车道起点，终点不超过车道终点）
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));

    // 将合并且裁剪后的区间加入输出的阻塞区间集合中
    block_range->push_back(std::move(range));
  }
}

/*
例如，假设阻塞区间是 [3,5] 和 [7,9]，车道起点0，终点10，则 all_value 为 [0,3,5,7,9,10]。

则有效区间就是 [0,3]、[5,7]、[9,10]，这些区间是可通行的
*/
/// @brief 根据传入的原始区间（origin_range），结合车道的起止位置，生成一个排好序且有效的区间集合（valid_range）
/// @param topo_node 
/// @param origin_range 
/// @param valid_range 
void GetSortedValidRange(const TopoNode* topo_node,
                         const std::vector<NodeSRange>& origin_range,
                         std::vector<NodeSRange>* valid_range) {
  // 用于存放合并后的“阻塞区间”
  std::vector<NodeSRange> block_range;
  // 对原始区间进行合并，得到一组合并后的区间，避免区间重叠和碎片化
  MergeBlockRange(topo_node, origin_range, &block_range);
  // 获取该拓扑节点的起点和终点位置的 s 值（车道上的起止坐标）
  double start_s = topo_node->StartS();
  double end_s = topo_node->EndS();

  // 构造一个数值序列 all_value： 用来从阻塞区间划分出有效区间的边界点
  // 包含： 起点 start_s  阻塞区间的所有起点和终点（交替加入） 终点 end_s
  std::vector<double> all_value;
  all_value.push_back(start_s);
  for (const auto& range : block_range) {
    all_value.push_back(range.StartS());
    all_value.push_back(range.EndS());
  }
  all_value.push_back(end_s);

  // 遍历 all_value，每两个点作为一对区间端点，构造一个 NodeSRange（有效区间）
  for (size_t i = 0; i < all_value.size(); i += 2) {
    NodeSRange new_range(all_value[i], all_value[i + 1]);
    // 将这些区间添加到输出参数 valid_range 中
    valid_range->push_back(std::move(new_range));
  }
}

bool IsReachable(const TopoNode* from_node, const TopoNode* to_node) {
  double start_s = to_node->StartS() / to_node->Length() * from_node->Length();
  start_s = std::max(start_s, from_node->StartS());
  double end_s = to_node->EndS() / to_node->Length() * from_node->Length();
  end_s = std::min(end_s, from_node->EndS());
  return (end_s - start_s > MIN_POTENTIAL_LANE_CHANGE_LEN);
}

}  // namespace

/*
根据黑名单区间提取出每个车道的有效子区间；
基于这些有效区间初始化对应的子节点；
初始化子节点之间的子边；
添加潜在的子边连接（可能是非直接相邻但逻辑上相连的）
*/
/// @brief 基于传入的“黑名单区间映射”（black_map）初始化子拓扑图（SubTopoGraph）的主要步骤
/// @param black_map 一个 unordered_map，key 是指向原始 TopoNode 的指针，value 是该车道对应的多个可用区间（NodeSRange）列表
SubTopoGraph::SubTopoGraph(
    const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
        black_map) {
  std::vector<NodeSRange> valid_range;
  // 遍历黑名单映射表里的每个原始车道节点及其区间
  for (const auto& map_iter : black_map) {
    valid_range.clear();
    // 对当前车道的多个区间进行排序和过滤，得到排好序且有效的区间集合 valid_range
    GetSortedValidRange(map_iter.first, map_iter.second, &valid_range);
    // 初始化这个车道对应的 子节点： 这里的“子节点”是指拓扑图中原节点的子分段，按区间划分成更细粒度的节点
    InitSubNodeByValidRange(map_iter.first, valid_range);
  }

  // 继续遍历黑名单映射表中的每个原始车道节点
  for (const auto& map_iter : black_map) {
  // 初始化它们之间的连接边（子边）：基于原拓扑图中车道节点的连接关系，将连接映射到子节点层级
    InitSubEdge(map_iter.first);
  }

  // 再次遍历每个原始车道节点
  for (const auto& map_iter : black_map) {
  // 添加“潜在的子边”，即可能的非标准连接，或为了更丰富路径规划考虑而加入的连接
  // 潜在边可以是跨车道跳转、特殊车道合流等路径规划需要的连接
    AddPotentialEdge(map_iter.first);
  }
}

SubTopoGraph::~SubTopoGraph() {}

void SubTopoGraph::GetSubInEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(to_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  for (const auto* sub_node : sub_nodes) {
    for (const auto* in_edge : sub_node->InFromAllEdge()) {
      if (in_edge->FromNode() == from_node) {
        sub_edges->insert(in_edge);
      }
    }
  }
}

void SubTopoGraph::GetSubOutEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(from_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  for (const auto* sub_node : sub_nodes) {
    for (const auto* out_edge : sub_node->OutToAllEdge()) {
      if (out_edge->ToNode() == to_node) {
        sub_edges->insert(out_edge);
      }
    }
  }
}

const TopoNode* SubTopoGraph::GetSubNodeWithS(const TopoNode* topo_node,
                                              double s) const {
  const auto& map_iter = sub_node_range_sorted_map_.find(topo_node);
  if (map_iter == sub_node_range_sorted_map_.end()) {
    return topo_node;
  }
  const auto& sorted_vec = map_iter->second;
  // sorted vec can't be empty!
  int index = BinarySearchForStartS(sorted_vec, s);
  if (index < 0) {
    return nullptr;
  }
  return sorted_vec[index].GetTopoNode();
}

// 根据给定的 valid_range（一组有效区间）把一个大 TopoNode 拆分成多个子节点 (sub_topo_node)，并且在这些子节点之间建立顺序连接的边
void SubTopoGraph::InitSubNodeByValidRange(
    const TopoNode* topo_node, const std::vector<NodeSRange>& valid_range) {
  // Attention: no matter topo node has valid_range or not,
  // create map value first;
  // 当前 topo_node 对应的所有子节点及其区间
  auto& sub_node_vec = sub_node_range_sorted_map_[topo_node];
  // 方便快速查找子节点的集合
  auto& sub_node_set = sub_node_map_[topo_node];

  std::vector<TopoNode*> sub_node_sorted_vec;
  // 遍历所有有效区间，生成子节点
  for (const auto& range : valid_range) {
    // 跳过过小的区间（小于 MIN_INTERNAL_FOR_NODE，避免生成碎片节点）
    if (range.Length() < MIN_INTERNAL_FOR_NODE) {
      continue;
    }
    std::shared_ptr<TopoNode> sub_topo_node_ptr;
    // 用原始的 topo_node 和这个区间构造一个新的子节点对象
    sub_topo_node_ptr.reset(new TopoNode(topo_node, range));
    // 将子节点指针和区间保存到对应的容器中
    sub_node_vec.emplace_back(sub_topo_node_ptr.get(), range);
    // topo_nodes_ 是整个子图管理的所有子节点的集合，保存所有子节点的 shared_ptr，保证生命周期
    sub_node_set.insert(sub_topo_node_ptr.get());
    // sub_node_sorted_vec 临时保存所有生成的子节点指针，用于下一步连接
    sub_node_sorted_vec.push_back(sub_topo_node_ptr.get());
    topo_nodes_.push_back(std::move(sub_topo_node_ptr));
  }
  
  // 连接相邻的子节点
  for (size_t i = 1; i < sub_node_sorted_vec.size(); ++i) {
    auto* pre_node = sub_node_sorted_vec[i - 1];
    auto* next_node = sub_node_sorted_vec[i];
    // 判断两个节点的区间是否“足够接近”（IsCloseEnough），避免出现间隙过大断开
    if (IsCloseEnough(pre_node->EndS(), next_node->StartS())) {
      // 如果接近，则构造一条从前一个子节点到后一个子节点的有向边，代表子节点之间的连接关系
      Edge edge;
      edge.set_from_lane_id(topo_node->LaneId());
      edge.set_to_lane_id(topo_node->LaneId());
      edge.set_direction_type(Edge::FORWARD);
      edge.set_cost(0.0);
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(new TopoEdge(edge, pre_node, next_node));
      // 将这条边加入前驱节点的出边和后继节点的入边列表
      pre_node->AddOutEdge(topo_edge_ptr.get());
      next_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

// 拿到原始节点的所有子节点后，为它们“继承”原始节点的入边和出边，建立起子节点之间对应的连接关系
//为给定的一个原始节点（topo_node）的所有子节点（sub_nodes）初始化其入边和出边
void SubTopoGraph::InitSubEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  // 获取该原始节点对应的所有子节点集合
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }
  
  // 遍历所有子节点，对每个子节点分别调用两个初始化边的方法
  for (auto* sub_node : sub_nodes) {
    // 根据原始节点所有的入边（InFromAllEdge()），给该子节点初始化入边
    InitInSubNodeSubEdge(sub_node, topo_node->InFromAllEdge());
    // 根据原始节点所有的出边（OutToAllEdge()），给该子节点初始化出边
    InitOutSubNodeSubEdge(sub_node, topo_node->OutToAllEdge());
  }
}

void SubTopoGraph::InitInSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        if (!sub_from_node->IsOverlapEnough(sub_node, in_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else if (in_edge->FromNode()->IsOverlapEnough(sub_node, in_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

void SubTopoGraph::InitOutSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (!sub_node->IsOverlapEnough(sub_to_node, out_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else if (sub_node->IsOverlapEnough(out_edge->ToNode(), out_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

/// @brief 根据给定的原始节点 node，从内部的 sub_node_map_ 容器里查找对应的所有子节点集合，并把这些子节点放到 sub_nodes 输出参数里
/// @param node 
/// @param sub_nodes 
/// @return 
bool SubTopoGraph::GetSubNodes(
    const TopoNode* node,
    std::unordered_set<TopoNode*>* const sub_nodes) const {
  // 在 sub_node_map_ 中查找 node 作为 key 的记录
  // sub_node_map_ 是一个从原始节点指针映射到该节点所有子节点集合的哈希表
  const auto& iter = sub_node_map_.find(node);
  if (iter == sub_node_map_.end()) {
    return false;
  }
  sub_nodes->clear();
  sub_nodes->insert(iter->second.begin(), iter->second.end());
  return true;
}

void SubTopoGraph::AddPotentialEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }
  for (auto* sub_node : sub_nodes) {
    AddPotentialInEdge(sub_node, topo_node->InFromLeftOrRightEdge());
    AddPotentialOutEdge(sub_node, topo_node->OutToLeftOrRightEdge());
  }
}

void SubTopoGraph::AddPotentialInEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        if (sub_node->GetInEdgeFrom(sub_from_node) != nullptr) {
          continue;
        }
        if (!IsReachable(sub_from_node, sub_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetInEdgeFrom(in_edge->FromNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

void SubTopoGraph::AddPotentialOutEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (sub_node->GetOutEdgeTo(sub_to_node) != nullptr) {
          continue;
        }
        if (!IsReachable(sub_node, sub_to_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetOutEdgeTo(out_edge->ToNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

}  // namespace routing
}  // namespace apollo
