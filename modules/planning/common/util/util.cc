/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/common/util/util.h"

#include <limits>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace util {

using apollo::common::VehicleState;
using apollo::hdmap::PathOverlap;
using apollo::routing::RoutingResponse;

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

bool IsDifferentRouting(const RoutingResponse& first,
                        const RoutingResponse& second) {
  if (first.has_header() && second.has_header()) {
    return first.header().sequence_num() != second.header().sequence_num();
  }
  return true;
}

double GetADCStopDeceleration(const double adc_front_edge_s,
                              const double stop_line_s) {
  double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed < max_adc_stop_speed) {
    return 0.0;
  }

  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

/*
 * @brief: check if a stop_sign_overlap is still along reference_line
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  return (stop_sign_overlap_it != stop_sign_overlaps.end());
}

/*
 * @brief: check if a traffic_light_overlap is still along reference_line
 */
bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it != traffic_light_overlaps.end());
}

/*
 * @brief: check if ADC is till inside a pnc-junction
 */
bool CheckInsidePnCJunction(const ReferenceLineInfo& reference_line_info) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  hdmap::PathOverlap pnc_junction_overlap;
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  if (pnc_junction_overlap.object_id.empty()) {
    return false;
  }

  static constexpr double kIntersectionPassDist = 2.0;  // unit: m
  const double distance_adc_pass_intersection =
      adc_back_edge_s - pnc_junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_intersection[" << distance_adc_pass_intersection
         << "] pnc_junction_overlap[" << pnc_junction_overlap.object_id
         << "] start_s[" << pnc_junction_overlap.start_s << "]";

  return distance_adc_pass_intersection < kIntersectionPassDist;
}

/*
 * @brief: get files at a path
 */
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files) {
  ACHECK(files);
  if (!boost::filesystem::exists(path)) {
    return;
  }
  if (boost::filesystem::is_regular_file(path)) {
    AINFO << "Found record file: " << path.c_str();
    files->push_back(path.c_str());
    return;
  }
  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      GetFilesByPath(entry.path(), files);
    }
  }
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
