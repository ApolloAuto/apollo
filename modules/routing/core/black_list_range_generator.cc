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

#include "modules/routing/core/black_list_range_generator.h"

namespace apollo {
namespace routing {

constexpr double S_GAP_FOR_BLACK = 0.01;

namespace {

double MoveSForward(double s, double upper_bound) {
  if (s > upper_bound) {
    AERROR << "Illegal s: " << s << ", upper bound: " << upper_bound;
    return s;
  }
  if (s + S_GAP_FOR_BLACK < upper_bound) {
    return (s + S_GAP_FOR_BLACK);
  } else {
    return ((s + upper_bound) / 2.0);
  }
}

double MoveSBackward(double s, double lower_bound) {
  if (s < lower_bound) {
    AERROR << "Illegal s: " << s << ", lower bound: " << lower_bound;
    return s;
  }
  // S_GAP_FOR_BLACK：是一个全局常量（通常 ~0.2m、0.3m，避免路径拼接过于紧密），表示黑名单裁剪的安全间距
  if (s - S_GAP_FOR_BLACK > lower_bound) {
    return (s - S_GAP_FOR_BLACK);
  } else {
    return ((s + lower_bound) / 2.0);
  }
}

/// @brief 递归地收集与给定节点 node 相邻的“平行”车道节点（拓扑节点），即从当前节点出发，沿着“向左或向右”的出边遍历，找到所有平行的节点并存入 node_set 中
/// @param node 
/// @param node_set 
void GetOutParallelLane(const TopoNode* node,
                        std::unordered_set<const TopoNode*>* const node_set) {
  // 遍历当前节点的所有“向左”或“向右”的出边                        
  for (const auto* edge : node->OutToLeftOrRightEdge()) {
  //取这条边的目标节点
    const auto* to_node = edge->ToNode();
    // 如果目标节点尚未加入集合，防止重复访问（防止死循环）
    if (node_set->count(to_node) == 0) {
      node_set->emplace(to_node); // 加入集合
      GetOutParallelLane(to_node, node_set);// 递归遍历目标节点的平行节点
    }
  }
}

// 从当前节点出发，沿着“入向左右平行边”（InFromLeftOrRightEdge()）递归访问，收集所有从左边或右边与当前车道平行，且连接到当前车道的上游车道节点
/// @brief 递归获取所有“入向”的平行车道节点
/// @param node 
/// @param node_set 
void GetInParallelLane(const TopoNode* node,
                       std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->InFromLeftOrRightEdge()) {
  // 取该入边对应的起始节点（上游节点）
    const auto* from_node = edge->FromNode();
    if (node_set->count(from_node) == 0) {
      node_set->emplace(from_node);
      GetInParallelLane(from_node, node_set);
    }
  }
}

// for new navigator
// 把 request 里的 黑名单道路，转换成 黑名单区间，放进 range_manager 里
void AddBlackMapFromRoad(const routing::RoutingRequest& request,
                         const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  // 获取路段上的所有节点
  for (const auto& road_id : request.blacklisted_road()) {
    std::unordered_set<const TopoNode*> road_nodes_set;
    // 通过 TopoGraph 的方法，把一个 road_id 映射到 这个路段上的所有车道节点
    graph->GetNodesByRoadId(road_id, &road_nodes_set);
    // 对每个节点，添加黑名单区间
    for (const auto& node : road_nodes_set) {
      // 对于该道路上的所有 TopoNode，把它 从头到尾（[0.0, node->Length()]）都标记为黑名单
      range_manager->Add(node, 0.0, node->Length());
    }
  }
}

// for new navigator
// 根据 RoutingRequest 中的黑名单车道，生成对应的黑名单范围，然后放到 range_manager 中
void AddBlackMapFromLane(const routing::RoutingRequest& request,
                         const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  // 每个元素（lane）通常有：id()：车道的唯一标识 start_s()、end_s()：黑名单范围的起点和终点
  for (const auto& lane : request.blacklisted_lane()) {
  // 根据车道 ID，去 拓扑图 TopoGraph 中找对应的 TopoNode
    const auto* node = graph->GetNode(lane.id());
    if (node) {
      range_manager->Add(node, lane.start_s(), lane.end_s());
    }
  }
}

// 根据给定的车道 node，将它的外侧平行车道上对应裁剪位置附近（由 cut_ratio 指定的比例点）标记为黑名单，用以避免规划时驶入这些区域
/// @brief 给定一个拓扑节点 node，基于它的“外侧平行车道”，在 range_manager 中添加对应的黑名单区间
/// @param node 当前的车道拓扑节点
/// @param cut_ratio 裁剪比例，0~1之间，表示在对应车道长度上的位置比例
/// @param range_manager 用来管理车道黑名单区间的管理器指针
void AddBlackMapFromOutParallel(const TopoNode* node, double cut_ratio,
                                TopoRangeManager* const range_manager) {
  // 无序集合 par_node_set，用于存放与当前车道 node 外侧平行的所有车道节点                               
  std::unordered_set<const TopoNode*> par_node_set;
  // 将当前车道的外侧平行车道节点放入 par_node_set
  GetOutParallelLane(node, &par_node_set);
  // 从集合中移除自己（node），防止自己重复加入黑名单
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    // 计算裁剪位置 par_cut_s，即该车道长度乘以裁剪比例 cut_ratio
    double par_cut_s = cut_ratio * par_node->Length();
    // 在 range_manager 中添加黑名单区间，这个区间是一个点区间（起点和终点都等于 par_cut_s），表示将此处标记为“黑名单
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

/// @brief 针对**“入向”的平行车道**进行处理
/// @param node 
/// @param cut_ratio 
/// @param range_manager 
void AddBlackMapFromInParallel(const TopoNode* node, double cut_ratio,
                               TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetInParallelLane(node, &par_node_set); // 获取所有入向的平行车道节点
  par_node_set.erase(node); // 去除自身节点，避免重复
  for (const auto* par_node : par_node_set) {
    // cut_ratio 是一个[0,1]的比例，表示当前车道节点上的位置占总长度的比例。乘以该节点长度获得对应的S坐标
    double par_cut_s = cut_ratio * par_node->Length(); // 计算对应的s位置
    range_manager->Add(par_node, par_cut_s, par_cut_s); // 只添加单点区间
  }
}

}  // namespace

/// @brief 根据 路由请求 中提供的信息（如黑名单车道、黑名单路段等），在 TopoRangeManager 中登记不可通行的范围
/// @param request 
/// @param graph 
/// @param range_manager 
void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const routing::RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  // 从请求中的 黑名单车道 添加不可通行范围
  AddBlackMapFromLane(request, graph, range_manager);
  // 从请求中的 黑名单道路 添加不可通行范围
  AddBlackMapFromRoad(request, graph, range_manager);
  // 对所有黑名单范围进行 排序并合并
  range_manager->SortAndMerge();
}

/// @brief 
/// @param src_node 起点（起始 Lane）
/// @param dest_node 终点（终止 Lane）
/// @param start_s 在起点的 s 坐标
/// @param end_s 在终点的 s 坐标
/// @param range_manager 用于管理黑名单范围（可行驶区域裁剪）
void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  // start_length 是起点 Lane 的长度
  double start_length = src_node->Length();
  // end_length 是终点 Lane 的长度
  double end_length = dest_node->Length();

// 定义了一个很小的数 0.01，作为“允许误差范围”
  static constexpr double kEpsilon = 1e-2;

  // 校正起点 s 值：如果 start_s 比起点 lane 长度略大（差值 ≤ 0.01），就直接当作 start_length
  const double start_s_adjusted =
      (start_s > start_length && start_s - start_length <= kEpsilon)
          ? start_length
          : start_s;
  // 校正终点 s 值： 如果 end_s 比终点 Lane 长度略大，就用 end_length
  const double end_s_adjusted =
      (end_s > end_length && end_s - end_length <= kEpsilon) ? end_length
                                                             : end_s;
  
  // 检查起点 s 是否合法
  if (start_s_adjusted < 0.0 || start_s_adjusted > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }
  // 检查终点 s 是否合法
  if (end_s_adjusted < 0.0 || end_s_adjusted > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  // 计算起点的黑名单裁剪位置
  // 调用 MoveSBackward()，把 start_s_adjusted 往“后”稍微移动，得到裁剪 s 值
  // 这里第二个参数 0.0 说明是移动到起点位置
  double start_cut_s = MoveSBackward(start_s_adjusted, 0.0);

  // 把起点裁剪点加入黑名单，在 src_node 里，把起点处的 s 范围裁掉（加到黑名单）
  range_manager->Add(src_node, start_cut_s, start_cut_s);

  // 处理起点的“并行段”黑名单，进一步把起点 lane 的并行段（起点部分）也加入黑名单
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);
  
  // 计算终点的黑名单裁剪位置
  // 调用 MoveSForward()，把 end_s_adjusted 往“前”稍微移动，得到裁剪 s
  double end_cut_s = MoveSForward(end_s_adjusted, end_length);
  // 把终点裁剪点加入黑名单，在终点 Lane 里，也把裁剪位置加到黑名单
  range_manager->Add(dest_node, end_cut_s, end_cut_s);
  // 处理终点的“并行段”黑名单，进一步把终点 lane 的并行段（入口部分）也加入黑名单
  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  // 最后合并范围，合并（排序 & 合并）所有已添加的黑名单范围，避免重叠或重复
  range_manager->SortAndMerge();
}

}  // namespace routing
}  // namespace apollo
