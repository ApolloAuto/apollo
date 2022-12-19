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
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/cyber.h"
#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/discrete_points_reference_line_smoother.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

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
  ReferenceLineProvider(
      const common::VehicleStateProvider* vehicle_state_provider,
      const hdmap::HDMap* base_map,
      const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr);

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  bool UpdateRoutingResponse(const routing::RoutingResponse& routing);

  void UpdateVehicleState(const common::VehicleState& vehicle_state);

  bool Start();

  void Stop();

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);

  double LastTimeDelay();

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints();

  bool UpdatedReferenceLine() { return is_reference_line_updated_.load(); }

 private:
  /**
   * @brief Use PncMap to create reference line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/
  bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                           std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines,
      const std::list<hdmap::RouteSegments>& route_segments);

  void GenerateThread();
  void IsValidReferenceLine();
  void PrioritzeChangeLane(std::list<hdmap::RouteSegments>* route_segments);

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

  std::unique_ptr<ReferenceLineSmoother> smoother_;
  ReferenceLineSmootherConfig smoother_config_;

  std::mutex pnc_map_mutex_;
  std::unique_ptr<hdmap::PncMap> pnc_map_;

  // Used in Navigation mode
  std::shared_ptr<relative_map::MapMsg> relative_map_;

  std::mutex vehicle_state_mutex_;
  common::VehicleState vehicle_state_;

  std::mutex routing_mutex_;
  routing::RoutingResponse routing_;
  bool has_routing_ = false;

  std::mutex reference_lines_mutex_;
  std::list<ReferenceLine> reference_lines_;
  std::list<hdmap::RouteSegments> route_segments_;
  double last_calculation_time_ = 0.0;

  std::queue<std::list<ReferenceLine>> reference_line_history_;
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;

  std::future<void> task_future_;

  std::atomic<bool> is_reference_line_updated_{true};

  const common::VehicleStateProvider* vehicle_state_provider_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
