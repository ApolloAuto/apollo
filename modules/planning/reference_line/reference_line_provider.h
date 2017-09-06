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
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"

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

  void Init(const hdmap::PncMap* pnc_map_,
            const routing::RoutingResponse& routing_response,
            const ReferenceLineSmootherConfig& smoother_config);

  bool Start();

  std::vector<ReferenceLine> GetReferenceLines();

 private:
  void Generate();
  void IsValidReferenceLine();
  bool CreateReferenceLineFromRouting(const common::PointENU& position,
                                      const routing::RoutingResponse& routing);

 private:
  DECLARE_SINGLETON(ReferenceLineProvider);

  bool is_initialized_ = false;
  std::unique_ptr<std::thread> thread_;

  const hdmap::PncMap* pnc_map_ = nullptr;
  routing::RoutingResponse routing_response_;
  ReferenceLineSmootherConfig smoother_config_;

  std::mutex reference_line_groups_mutex_;
  std::list<std::vector<ReferenceLine>> reference_line_groups_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
