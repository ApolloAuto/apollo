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

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modules/common/proto/vehicle_state.pb.h"

#include "modules/common/util/util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
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
  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  void Init(const hdmap::HDMap* hdmap_,
            const QpSplineReferenceLineSmootherConfig& smoother_config);

  bool UpdateRoutingResponse(const routing::RoutingResponse& routing);

  void UpdateVehicleState(const common::VehicleState& vehicle_state);

  bool Start();

  void Stop();

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief Use PncMap to create refrence line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/
  bool CreateReferenceLineFromRouting(
      std::list<ReferenceLine>* reference_lines,
      std::list<hdmap::RouteSegments>* segments);

 private:
  void GenerateThread();
  void IsValidReferenceLine();
  void PrioritzeChangeLane(std::list<hdmap::RouteSegments>* route_segments);
  bool IsAllowChangeLane(const common::math::Vec2d& point,
                         const std::list<hdmap::RouteSegments>& route_segments);

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                  const ReferenceLine& smoothed) const;

  bool SmoothReferenceLine(const hdmap::RouteSegments& lanes,
                           ReferenceLine* reference_line);

 private:
  DECLARE_SINGLETON(ReferenceLineProvider);

  bool is_initialized_ = false;
  std::unique_ptr<std::thread> thread_;

  std::unique_ptr<ReferenceLineSmoother> smoother_;

  std::mutex pnc_map_mutex_;
  std::unique_ptr<hdmap::PncMap> pnc_map_;
  common::VehicleState vehicle_state_;

  bool has_routing_ = false;

  QpSplineReferenceLineSmootherConfig smoother_config_;

  bool is_stop_ = false;

  std::mutex reference_lines_mutex__;
  std::condition_variable cv_has_reference_line_;
  std::list<ReferenceLine> reference_lines_;
  std::list<hdmap::RouteSegments> route_segments_;

  struct SegmentHistory {
    double min_l = 0.0;
    double accumulate_s = 0.0;
    common::math::Vec2d last_point;
  };
  std::unordered_map<std::string, SegmentHistory> segment_history_;

  std::unique_ptr<Spline2dSolver> spline_solver_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
