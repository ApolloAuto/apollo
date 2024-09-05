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

#pragma once

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {
namespace util {

bool IsVehicleStateValid(const apollo::common::VehicleState& vehicle_state);

bool IsDifferentRouting(const PlanningCommand& first,
                        const PlanningCommand& second);

double GetADCStopDeceleration(
    apollo::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s);

bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id);

bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id);

bool CheckInsideJunction(const ReferenceLineInfo& reference_line_info);

void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files);

double CalculateEquivalentEgoWidth(const ReferenceLineInfo& reference_line_info,
                                   double s, bool* is_left);

double CalculateEquivalentEgoWidth(
    const apollo::hdmap::LaneInfoConstPtr lane_info, double s, bool* is_left);

bool left_arc_bound_with_heading(double delta_x, double r, double heading,
                                 double* result);
bool right_arc_bound_with_heading(double delta_x, double r, double heading,
                                  double* result);
bool left_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                    double heading,
                                                    double kappa,
                                                    double* result);
bool right_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                     double heading,
                                                     double kappa,
                                                     double* result);

}  // namespace util
}  // namespace planning
}  // namespace apollo
