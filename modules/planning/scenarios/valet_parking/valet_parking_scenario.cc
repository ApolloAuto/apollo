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

/**
 * @file
 **/

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario,
                                          const Frame& frame) {
  // TODO(all) Implement available parking spot detection by preception results
  if (!frame.local_view().planning_command->has_parking_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  std::string target_parking_spot_id;
  if (frame.local_view().planning_command->has_parking_command() &&
      frame.local_view()
          .planning_command->parking_command()
          .has_parking_spot_id()) {
    target_parking_spot_id = frame.local_view()
                                 .planning_command->parking_command()
                                 .parking_spot_id();
  } else {
    ADEBUG << "No parking space id from routing";
    return false;
  }

  if (target_parking_spot_id.empty()) {
    return false;
  }

  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  PathOverlap parking_space_overlap;
  const auto& vehicle_state = frame.vehicle_state();

  if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id,
                                     &parking_space_overlap)) {
    ADEBUG << "No such parking spot found after searching all path forward "
              "possible"
           << target_parking_spot_id;
    return false;
  }
  double parking_spot_range_to_start =
      context_.scenario_config.parking_spot_range_to_start();
  if (!CheckDistanceToParkingSpot(frame, vehicle_state, nearby_path,
                                  parking_spot_range_to_start,
                                  parking_space_overlap)) {
    ADEBUG << "target parking spot found, but too far, distance larger than "
              "pre-defined distance"
           << target_parking_spot_id;
    return false;
  }
  context_.target_parking_spot_id = target_parking_spot_id;
  return true;
}

bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_id) {
      *parking_space_overlap = parking_overlap;
      return true;
    }
  }
  return false;
}

bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const Path& nearby_path, const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // TODO(Jinyun) parking overlap s are wrong on map, not usable
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  double center_point_s, center_point_l;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
  Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
  Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
  Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
  Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
  Vec2d center_point = (left_bottom_point + right_bottom_point +
                        right_top_point + left_top_point) /
                       4.0;
  nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
