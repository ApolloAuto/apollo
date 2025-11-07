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

#include "modules/routing/core/navigator.h"

#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using apollo::common::ErrorCode;

bool ShowRequestInfo(const routing::RoutingRequest& request,
                     const TopoGraph* graph) {
  // 遍历请求中的所有路径点（waypoint）
  for (const auto& wp : request.waypoint()) {

    // 检查每个 waypoint 是否存在
    const auto* node = graph->GetNode(wp.id());
    if (node == nullptr) {
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      return false;
    }
    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
  }

// 遍历 Blacklisted Lanes
  for (const auto& bl : request.blacklisted_lane()) {

    // 检查和打印黑名单车道信息
    const auto* node = graph->GetNode(bl.id());
    if (node == nullptr) {
      AERROR << "Black list node is not found in topo graph! ID: " << bl.id();
      return false;
    }
    // 日志打印每个 waypoint 信息：包括车道 ID、s 值、位置信息 (x, y)，以及车道长度
    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
  }

// 如果所有 waypoints 和 blacklisted_lanes 都能在图里找到，返回 true
  return true;
}

/// @brief 
/// @param request 用户的路由请求，包含一系列 waypoint
/// @param graph 全局道路拓扑结构的 TopoGraph
/// @param way_nodes 对应于 waypoint 的 TopoNode 节点指针列表
/// @param way_s 每个 waypoint 在其所属 lane 上的纵向坐标（s）
/// @return 
bool GetWayNodes(const routing::RoutingRequest& request,
                 const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  for (const auto& point : request.waypoint()) {
    // point.id()：每个 waypoint 中的 lane id（对应车道 ID）
    // 从 TopoGraph 的索引里查找这个 lane id 对应的 TopoNode
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }

    // way_nodes：保存对应的 TopoNode 节点指针
    way_nodes->push_back(cur_node);
    // way_s：保存该 waypoint 在 lane 上的纵向坐标 s
    way_s->push_back(point.s());
  }
  return true;
}

void SetErrorCode(const common::ErrorCode& error_code_id,
                  const std::string& error_string,
                  common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (error_code_id == common::ErrorCode::OK) {
    ADEBUG << error_string.c_str();
  } else {
    AERROR << error_string.c_str();
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    AINFO << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS();
  }
}

}  // namespace

/// @brief 通过拓扑图文件初始化导航器对象
/// @param topo_file_path 
Navigator::Navigator(const std::string& topo_file_path) {
  // 存储从文件加载的拓扑图数据
  Graph graph;
  if (!cyber::common::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    return;
  }

// 创建一个新的 TopoGraph 对象，并用智能指针 graph_ 管理
  graph_.reset(new TopoGraph());
  // 加载边以及节点
  if (!graph_->LoadGraph(graph)) {
    AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
    return;
  }

  // 创建并初始化 BlackListRangeGenerator 对象，用于生成“黑名单”区域（通常是路由中需避开的区域）
  black_list_generator_.reset(new BlackListRangeGenerator);

  // 创建并初始化 ResultGenerator 对象，用于生成路由结果（路径规划的最终结果）
  result_generator_.reset(new ResultGenerator);

  // 标记导航器状态为“已准备好”，表示初始化成功，可以开始执行路由任务
  is_ready_ = true;
  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const { return is_ready_; }

void Navigator::Clear() { topo_range_manager_.Clear(); }

/// @brief 
/// @param request 路由请求（包含起点、终点和途经点）
/// @param graph 拓扑图指针
/// @param way_nodes 从请求 waypoint 中解析到的拓扑节点（TopoNode）列表
/// @param way_s 对应节点在 lane 上的纵向坐标（s 坐标）
/// @return 
bool Navigator::Init(const routing::RoutingRequest& request,
                     const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  // 清空导航器内部上一条规划残留的数据（如黑名单、上次结果等）                    
  Clear();

  //解析 Waypoint，生成搜索起止点
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  for (const auto& node : node_vec) {
    if (result_node_vec->empty() ||
        result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {
      result_node_vec->push_back(node);
    } else {
      if (result_node_vec->back().EndS() < node.StartS()) {
        AERROR << "Result route is not continuous.";
        return false;
      }
      result_node_vec->back().SetEndS(node.EndS());
    }
  }
  return true;
}


/// @brief 给定一系列路点（way_nodes 和它们的 s），分段 进行路径搜索，每段使用 AStarStrategy，最终合并成完整的路径
/// @param graph 
/// @param way_nodes 
/// @param way_s 
/// @param result_nodes 
/// @return 
bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  // 初始化搜索策略
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));
  
  // result_nodes：最终结果
  result_nodes->clear();
  // node_vec：分段搜索的所有结果，后续再做合并
  std::vector<NodeWithRange> node_vec;

  // 遍历 相邻路点对，逐段搜索路径
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];
    
    // 生成 full_range_manager（包含黑名单）：后续会裁剪
    // topo_range_manager_：可行驶范围
    TopoRangeManager full_range_manager = topo_range_manager_;
    // AddBlackMapFromTerminal：把这段的黑名单也加进去，避免搜索到不合法路段
    // 针对路径段的起点和终点做“黑名单范围”裁剪（可行驶范围限制），并且还包括它们的“并行段”拓展
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);
    
    // 构建 子图（SubTopoGraph）？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
    // 以范围（RangeMap）生成一个子图（只包含当前段落可用的路段）
    SubTopoGraph sub_graph(full_range_manager.RangeMap());

    // 获取 子图中的起点、终点
    // GetSubNodeWithS：保证起点/终点位于可行驶段中（可处理 s 范围）
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    // 在子图中使用 A 路径搜索
    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }
    
    // 拼接段落路径
    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }
   
   // 合并分段路径结果
  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

bool Navigator::SearchRoute(const routing::RoutingRequest& request,
                            routing::RoutingResponse* const response) {
  // 检查请求的有效性
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }

// Init完成后
// 检查规划器是否就绪：如果 Navigator 内部（比如拓扑图等）尚未加载完毕，也直接返回
  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }

  // 初始化导航请求
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  // 将 request 中的 waypoint 转换成实际的图中的 TopoNode 指针
  // 计算对应 s 坐标
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }

  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }
  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }

  // 修正首尾的 s 坐标：修正第一段和最后一段的起止 s，和请求 waypoint 对齐
  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());


// 生成最终可通行区域（PassageRegion）
// 过搜索出来的 result_nodes，结合地图信息，生成完整的可通行区域（PassageRegion）
// 在 response 中填充所有可行段落的 LaneSegment，最终生成完整的 RoutingResponse
  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }
  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo
