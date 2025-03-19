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

/**
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#pragma once

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/planning_msgs/navigation.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "cyber/cyber.h"   // 用于异步任务管理
#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"  // 提供车辆状态数据
#include "modules/map/pnc_map/pnc_map_base.h"   // 处理地图相关信息
#include "modules/planning/planning_base/common/indexed_queue.h"
#include "modules/planning/planning_base/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/planning_base/reference_line/discrete_points_reference_line_smoother.h"  // 离散点参考线平滑器
#include "modules/planning/planning_base/reference_line/qp_spline_reference_line_smoother.h"   // QP 样条曲线平滑器
#include "modules/planning/planning_base/reference_line/reference_line.h"  // 参考线数据结构
#include "modules/planning/planning_base/reference_line/spiral_reference_line_smoother.h"   // 螺旋线参考线平滑器

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
class ReferenceLineProvider {
 public:
  ReferenceLineProvider() = default;

  /// @brief 
  /// @param vehicle_state_provider 车辆状态提供器
  /// @param reference_line_config 参考线配置
  /// @param relative_map 可选：相对地图数据（用于导航模式）
  ReferenceLineProvider(
      const common::VehicleStateProvider* vehicle_state_provider,
      const ReferenceLineConfig* reference_line_config,
      const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr);

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  /**
   * @brief Update when new PlanningCommand is received.
   * @param command New PlanningCommand.
   * @return True if no error occurs.
   **/
  bool UpdatePlanningCommand(const planning::PlanningCommand& command);

  void UpdateVehicleState(const common::VehicleState& vehicle_state);
  
  // 启动参考线提供器（如果 FLAGS_enable_reference_line_provider_thread 为 true，则会创建一个线程）
  bool Start();
  
  // 停止参考线提供器
  void Stop();
  
  // 重置参考线提供器
  void Reset();
  
  // 获取参考线路和路由段
  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);
  
  // 获取上次计算的时间延迟
  double LastTimeDelay();
  
  // 获取未来的路由航点
  std::vector<routing::LaneWaypoint> FutureRouteWaypoints();
  
  // 检查参考线是否更新
  bool UpdatedReferenceLine() { return is_reference_line_updated_.load(); }
  
  // 获取终点车道信息
  void GetEndLaneWayPoint(
      std::shared_ptr<routing::LaneWaypoint>& end_point) const;
  
  // 根据车道 ID 获取车道信息
  hdmap::LaneInfoConstPtr GetLaneById(const hdmap::Id& id) const;

 private:
  /**
   * @brief Use LaneFollowMap to create reference line and the corresponding
   * segments based on routing and current position. This is a thread safe
   * function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/
  bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                           std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  // 更新参考线（如果数据未发生变化，则避免不必要的拷贝）
  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines,
      const std::list<hdmap::RouteSegments>& route_segments);
  
  // 生成参考线的后台线程
  void GenerateThread();
  // 检查参考线是否有效
  void IsValidReferenceLine();
  // 优先处理换道
  void PrioritizeChangeLane(std::list<hdmap::RouteSegments>* route_segments);

  bool CreateRouteSegments(const common::VehicleState& vehicle_state,
                           std::list<hdmap::RouteSegments>* segments);

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                  const ReferenceLine& smoothed) const;

  bool SmoothReferenceLine(const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line);

  bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,
                                   const ReferenceLine& raw_ref,
                                   ReferenceLine* reference_line);

  void GetAnchorPoints(const ReferenceLine& reference_line,
                       std::vector<AnchorPoint>* anchor_points) const;

  bool SmoothRouteSegment(const hdmap::RouteSegments& segments,
                          ReferenceLine* reference_line);

  /**
   * @brief This function creates a smoothed forward reference line
   * based on the given segments.
   */
  bool ExtendReferenceLine(const common::VehicleState& state,
                           hdmap::RouteSegments* segments,
                           ReferenceLine* reference_line);

  AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                             double s) const;

  bool GetReferenceLinesFromRelativeMap(
      std::list<ReferenceLine>* reference_lines,
      std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief This function get adc lane info from navigation path and map
   * by vehicle state.
   */
  bool GetNearestWayPointFromNavigationPath(
      const common::VehicleState& state,
      const std::unordered_set<std::string>& navigation_lane_ids,
      hdmap::LaneWaypoint* waypoint);

  bool Shrink(const common::SLPoint& sl, ReferenceLine* ref,
              hdmap::RouteSegments* segments);

 private:
  bool is_initialized_ = false;
  std::atomic<bool> is_stop_{false};
  
  // 参考线平滑器
  std::unique_ptr<ReferenceLineSmoother> smoother_;
  // 平滑器配置
  ReferenceLineSmootherConfig smoother_config_;

  std::mutex pnc_map_mutex_;
  // The loaded pnc map plugin which can create referene line from
  // PlanningCommand.
  // 多个 PncMapBase 实例
  std::vector<std::shared_ptr<planning::PncMapBase>> pnc_map_list_;
  // 当前使用的 PncMapBase
  std::shared_ptr<planning::PncMapBase> current_pnc_map_;

  // Used in Navigation mode
  std::shared_ptr<relative_map::MapMsg> relative_map_;

  std::mutex vehicle_state_mutex_;
  common::VehicleState vehicle_state_;

  std::mutex routing_mutex_;
  planning::PlanningCommand planning_command_;
  bool has_planning_command_ = false;
  bool is_new_command_ = false;

  std::mutex reference_lines_mutex_;
  std::list<ReferenceLine> reference_lines_;
  std::list<hdmap::RouteSegments> route_segments_;
  double last_calculation_time_ = 0.0;

  // 存储历史参考线
  std::queue<std::list<ReferenceLine>> reference_line_history_;
  // 存储历史路由段
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;
  
  // task_future_：用于异步任务执行
  std::future<void> task_future_;
  
  // is_reference_line_updated_：标志参考线是否已更新
  std::atomic<bool> is_reference_line_updated_{true};
  
  // vehicle_state_provider_：车辆状态提供器指针
  const common::VehicleStateProvider* vehicle_state_provider_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
