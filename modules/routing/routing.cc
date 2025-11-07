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

#include "modules/routing/routing.h"

#include <limits>
#include <unordered_map>

#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

using apollo::common::ErrorCode;
using apollo::common::PointENU;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

/// @brief 路由模块的初始化
/// @return 
apollo::common::Status Routing::Init() {
  // 获取路由用的拓扑图文件路径，RoutingMapFile() 通常返回一个包含拓扑信息的地图文件路径字符串。
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;

  // 创建并初始化 Navigator 对象，传入拓扑图路径
  // Navigator 是负责路径查找、导航决策的核心类，基于拓扑图执行路由算法
  navigator_ptr_.reset(new Navigator(routing_map_file));

  // 基础地图包含详细的车道线、交通标志等信息
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  return apollo::common::Status::OK();
}

apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}

/// @brief 补全或扩展 RoutingRequest 中缺失的车道信息（lane_id 和 s 坐标），同时生成额外候选请求
/// @param routing_request 一个原始的 RoutingRequest，可能部分 waypoint 没有 lane_id
/// @return 一组新的 RoutingRequest，每条请求都有完整的 lane_id 信息（以及额外候选车道生成的新请求）
std::vector<routing::RoutingRequest> Routing::FillLaneInfoIfMissing(
    const routing::RoutingRequest& routing_request) {
  // 最终结果
  std::vector<routing::RoutingRequest> fixed_requests;
  // 记录候选车道，索引是 waypoint 的下标
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;
  // 拷贝一份，用于补齐当前请求
  routing::RoutingRequest fixed_request(routing_request);

  // 遍历所有 Waypoints
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    if (lane_waypoint.has_id()) {
      continue;
    }

     //如果该 waypoint 已经有 lane_id，开始寻找最近的 lane
    // fill lane info when missing
    // 查找最近车道（多圈扩大半径搜索）
    // 将 waypoint 转成 ENU 坐标系的点
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    // 从 0.3m 半径开始，循环扩大半径（步进 0.3m，共 20 次，最大 6m）查找附近的 lanes
    constexpr double kRadius = 0.3;
    for (int i = 0; i < 20; ++i) {
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      if (lanes.size() > 0) {
        break;
      }
    }

    // 处理找不到 lane 的情况
    if (lanes.empty()) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return fixed_requests;  // return empty vector
    }

    // 记录匹配到的 lanes
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);
      if (j == 0) {
        //对于第一个匹配到的 lane： 直接更新到 fixed_request 的 waypoint
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        // 对于后续的 lane：生成一个新的 LaneWaypoint，放到 additional_lane_waypoint_map 中，表示这是该 waypoint 的候选车道（重叠部分）
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }
    }
  }

  // 生成所有可能的“请求组合”
  // first routing_request
  // 先把基础补齐的 fixed_request 放入结果列表
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
  // 遍历 additional_lane_waypoint_map
  // 按照候选车道，克隆当前所有 fixed_requests 并在对应 waypoint 中替换新的车道信息，形成新的请求
  // 逐步扩展请求组合，考虑所有多车道重叠的可能性
  for (const auto& m : additional_lane_waypoint_map) {
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        routing::RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }

  for (const auto& fixed_request : fixed_requests) {
    ADEBUG << "Fixed routing request:" << fixed_request.DebugString();
  }
  return fixed_requests;
}

/// @brief 
/// @param routing_request 来自用户的路径请求
/// @param routing_response 输出结果，存放路径规划成功后的完整路径
/// @return 
bool Routing::Process(
    const std::shared_ptr<routing::RoutingRequest>& routing_request,
    routing::RoutingResponse* const routing_response) {
  // 如果请求为空，发出警告并提前返回 true
  if (nullptr == routing_request) {
    AWARN << "Routing request is empty!";
    return true;
  }
  // 指针指向了一段内存
  CHECK_NOTNULL(routing_response);
  // 输出完整路径请求信息，方便调试或日志查看
  AINFO << "Get new routing request:" << routing_request->DebugString();

// 处理请求点缺失车道线信息的情况：补全每个请求点的 lane_id
// 如果请求中只有 GPS 或路径点，没有 lane_id，这个函数会通过 HDMap 反查补齐
  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);

  // 遍历所有补全后的请求，尝试规划路径
  double min_routing_length = std::numeric_limits<double>::max();
  for (const auto& fixed_request : fixed_requests) {
    routing::RoutingResponse routing_response_temp;
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
  // 如果搜索成功，比较其路径长度
  // 保留最短路径作为最终的 routing_response
      const double routing_length =
          routing_response_temp.measurement().distance();
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
  }


  if (min_routing_length < std::numeric_limits<double>::max()) {
    monitor_logger_buffer_.INFO("Routing success!");
    return true;
  }

  AERROR << "Failed to search route with navigator.";
  monitor_logger_buffer_.WARN("Routing failed! " +
                              routing_response->status().msg());
  return false;
}

}  // namespace routing
}  // namespace apollo
