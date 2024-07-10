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

// 初始化函数
apollo::common::Status Routing::Init() {
  // 读取拓扑地图routing_map的文件位置信息
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  // 在Navigator类加载指定的graph图
  navigator_ptr_.reset(new Navigator(routing_map_file));

  // 通过map模块提供的功能包,读取原始地图信息,即包括点和边的信息
  // 据此查找routing request请求的点距离最近的lane,并且返回对应的lane id.
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

std::vector<routing::RoutingRequest> Routing::FillLaneInfoIfMissing(
    const routing::RoutingRequest& routing_request) {
  std::vector<routing::RoutingRequest> fixed_requests;
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;
  routing::RoutingRequest fixed_request(routing_request);
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    if (lane_waypoint.has_id()) {
      continue;
    }

    // fill lane info when missing
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    constexpr double kRadius = 0.3;
    for (int i = 0; i < 20; ++i) {
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      if (lanes.size() > 0) {
        break;
      }
    }
    if (lanes.empty()) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return fixed_requests;  // return empty vector
    }
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);
      if (j == 0) {
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }
    }
  }
  // first routing_request
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
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

// 处理路由请求并返回相应的路由响应
bool Routing::Process(
    // 该函数接受一个指向RoutingRequest对象的智能指针作为参数
    const std::shared_ptr<routing::RoutingRequest>& routing_request,
    // 该函数返回一个指向RoutingResponse对象的指针
    routing::RoutingResponse* const routing_response) {
  // 首先检查传入的路由请求是否为空
  if (nullptr == routing_request) {
    AWARN << "Routing request is empty!";
    // 如果为空，返回true并结束函数
    return true;
  }
  // 接着检查传入的路由响应是否为空
  CHECK_NOTNULL(routing_response);
  // 记录接收到的路由请求
  AINFO << "Get new routing request:" << routing_request->DebugString();

  // 填充缺失的车道信息，并返回一组新的路由请求
  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);
  // 定义一个变量来存储最短的路由长度
  double min_routing_length = std::numeric_limits<double>::max();
  // 迭代处理每一个新的路由请求
  for (const auto& fixed_request : fixed_requests) {
    // 为每一个新的路由请求创建一个路由响应
    routing::RoutingResponse routing_response_temp;
    // 在导航器中搜索路由并将结果存储在临时路由响应中
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
      // 计算临时路由响应的路由长度
      const double routing_length =
          routing_response_temp.measurement().distance();
      // 如果路由长度小于目前存储的最短路由长度，则更新最短路由长度和路由响应
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
  }
  // 如果找到了最短的路由，则记录路由成功并返回true
  if (min_routing_length < std::numeric_limits<double>::max()) {
    monitor_logger_buffer_.INFO("Routing success!");
    return true;
  }

  // 如果没有找到最短的路由，则记录路由失败并返回false
  AERROR << "Failed to search route with navigator.";
  monitor_logger_buffer_.WARN("Routing failed! " +
                              routing_response->status().msg());
  return false;
}

}  // namespace routing
}  // namespace apollo
