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

#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

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

  void UpdateRoutingResponse(const routing::RoutingResponse& routing);

  bool UpdateVehicleStatus(const common::PointENU& position, double speed);

  bool Start();

  void Stop();

  bool HasReferenceLine();

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);

 private:
  void Generate();
  void IsValidReferenceLine();
  bool CreateReferenceLineFromRouting();

 private:
  DECLARE_SINGLETON(ReferenceLineProvider);

  bool is_initialized_ = false;
  std::unique_ptr<std::thread> thread_;

  std::mutex pnc_map_mutex_;
  std::unique_ptr<hdmap::PncMap> pnc_map_;
  double vehicle_speed_ = 0.0;

  bool has_routing_ = false;

  QpSplineReferenceLineSmootherConfig smoother_config_;

  bool is_stop_ = false;

  std::mutex reference_line_groups_mutex_;
  std::list<std::vector<ReferenceLine>> reference_line_groups_;
  std::list<std::vector<hdmap::RouteSegments>> route_segment_groups_;

  std::unique_ptr<Spline2dSolver> spline_solver_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
