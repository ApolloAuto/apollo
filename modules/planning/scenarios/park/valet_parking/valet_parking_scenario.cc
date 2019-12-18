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

#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"

#include "modules/planning/scenarios/park/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/park/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    ValetParkingScenario::s_stage_factory_;

void ValetParkingScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
}

void ValetParkingScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::VALET_PARKING_APPROACHING_PARKING_SPOT,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageApproachingParkingSpot(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::VALET_PARKING_PARKING,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageParking(config);
      });
}

std::unique_ptr<Stage> ValetParkingScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool ValetParkingScenario::GetScenarioConfig() {
  if (!config_.has_valet_parking_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.valet_parking_config());
  return true;
}

bool ValetParkingScenario::IsTransferable(const Frame& frame,
                                          const double parking_start_range) {
  // TODO(all) Implement available parking spot detection by preception results
  std::string target_parking_spot_id;
  if (frame.local_view().routing->routing_request().has_parking_space() &&
      frame.local_view().routing->routing_request().parking_space().has_id()) {
    target_parking_spot_id =
        frame.local_view().routing->routing_request().parking_space().id().id();
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

  if (!CheckDistanceToParkingSpot(vehicle_state, nearby_path,
                                  parking_start_range, parking_space_overlap)) {
    ADEBUG << "target parking spot found, but too far, distance larger than "
              "pre-defined distance"
           << target_parking_spot_id;
    return false;
  }

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
    const VehicleState& vehicle_state, const Path& nearby_path,
    const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // TODO(Jinyun) parking overlap s are wrong on map, not usable
  // double parking_space_center_s =
  //     (parking_space_overlap.start_s + parking_space_overlap.end_s) / 2.0;
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
  Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
  Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
  double left_bottom_point_s = 0.0;
  double left_bottom_point_l = 0.0;
  double right_bottom_point_s = 0.0;
  double right_bottom_point_l = 0.0;
  nearby_path.GetNearestPoint(left_bottom_point, &left_bottom_point_s,
                              &left_bottom_point_l);
  nearby_path.GetNearestPoint(right_bottom_point, &right_bottom_point_s,
                              &right_bottom_point_l);
  double parking_space_center_s =
      (left_bottom_point_s + right_bottom_point_s) / 2.0;
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs(parking_space_center_s - vehicle_point_s) <
      parking_start_range) {
    return true;
  } else {
    return false;
  }
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
