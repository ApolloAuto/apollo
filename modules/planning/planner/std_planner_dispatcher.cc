/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planner/std_planner_dispatcher.h"

// for open space planner
#include <vector>

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"

// for open space planner
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"

namespace apollo {
namespace planning {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneSegment;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;

std::unique_ptr<Planner> StdPlannerDispatcher::DispatchPlanner() {
  PlanningConfig planning_config;
  apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                         &planning_config);
  if (FLAGS_open_space_planner_switchable) {
    return planner_factory_.CreateObject(
        planning_config.standard_planning_config().planner_type(1));
  }
  return planner_factory_.CreateObject(
      planning_config.standard_planning_config().planner_type(0));
}

bool StdPlannerDispatcher::CheckParkingROI(
    ParkingSpaceInfoConstPtr target_parking_spot, const hdmap::HDMap* hdmap,
    const common::VehicleState& vehicle_state,
    std::vector<apollo::hdmap::LaneSegment>* segments_vector) {
  std::unique_ptr<Path> path_ =
      std::unique_ptr<Path>(new Path(*segments_vector));
  const auto& parking_space_overlaps = path_->parking_space_overlaps();
  if (parking_space_overlaps.size() != 0) {
    for (const auto& parking_overlap : parking_space_overlaps) {
      if (parking_overlap.object_id == FLAGS_target_parking_spot_id) {
        hdmap::Id id;
        id.set_id(parking_overlap.object_id);
        target_parking_spot = hdmap->GetParkingSpaceById(id);
      }
    }
  }
  if (target_parking_spot == nullptr) {
    std::string msg("No such parking spot found ");
    ADEBUG << msg << FLAGS_target_parking_spot_id;
    (*segments_vector).clear();
    return false;
  } else {
    Vec2d left_bottom_point = target_parking_spot->polygon().points().at(0);
    Vec2d right_bottom_point = target_parking_spot->polygon().points().at(1);
    double left_bottom_point_s = 0.0;
    double left_bottom_point_l = 0.0;
    double right_bottom_point_s = 0.0;
    double right_bottom_point_l = 0.0;
    double vehicle_point_s = 0.0;
    double vehicle_point_l = 0.0;
    path_->GetNearestPoint(left_bottom_point, &left_bottom_point_s,
                           &left_bottom_point_l);
    path_->GetNearestPoint(right_bottom_point, &right_bottom_point_s,
                           &right_bottom_point_l);
    Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
    path_->GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
    if (std::abs((left_bottom_point_s + right_bottom_point_s) / 2 -
                 vehicle_point_s) < FLAGS_parking_start_range) {
      return true;
    } else {
      return false;
    }
  }
}
}  // namespace planning
}  // namespace apollo
