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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {

class StopSignUnprotectedScenario : public Scenario {
  typedef std::unordered_map<std::string, std::vector<std::string>>
      StopSignLaneVehicles;

 public:
  StopSignUnprotectedScenario() :
    Scenario(ScenarioConfig::STOP_SIGN_UNPROTECTED) {}
  virtual ~StopSignUnprotectedScenario() = default;

  bool Init() override;

  void Observe(Frame* const frame);

  common::Status Process(const common::TrajectoryPoint& planning_init_point,
                         Frame* frame) override;

  bool IsTransferable(const Scenario& current_scenario,
                      const common::TrajectoryPoint& ego_point,
                      const Frame& frame) const override;

 private:
  enum class StopSignUnprotectedStage {
    UNKNOWN,
    PRE_STOP,
    STOP,
    CREEP,
    INTERSECTION_CRUISE,
  };

  void RegisterTasks();

  int StageIndexInConf(const StopSignUnprotectedStage& stage);

  StopSignUnprotectedStage GetNextStage(
      const StopSignUnprotectedStage& current_stage);

  common::Status PreStop(const ReferenceLineInfo& reference_line_info,
                         Frame* frame);
  common::Status Stop(const ReferenceLineInfo& reference_line_info,
                      Frame* frame);
  common::Status Creep(const ReferenceLineInfo& reference_line_info,
                       Frame* frame);
  common::Status IntersectionCruise(
      const common::TrajectoryPoint& planning_start_point,
      const ReferenceLineInfo& reference_line_info,
      Frame* frame);

  bool FindNextStopSign(const ReferenceLineInfo& reference_line_info);
  int GetAssociatedLanes(const hdmap::StopSignInfo& stop_sign_info);
  int AddWatchVehicle(
      const PathObstacle& path_obstacle,
      StopSignLaneVehicles* watch_vehicles);
  int RemoveWatchVehicle(const PathObstacle& path_obstacle,
                         const std::vector<std::string>& watch_vehicle_ids,
                         StopSignLaneVehicles* watch_vehicles);
  bool CheckADCStop(const ReferenceLineInfo& reference_line_info);

 private:
  std::vector<std::unique_ptr<Task>> tasks_;
  ScenarioConfig config_;
  StopSignUnprotectedStage stage_ = StopSignUnprotectedStage::PRE_STOP;
  bool stage_init_ = false;
  SpeedProfileGenerator speed_profile_generator_;
  hdmap::PathOverlap next_stop_sign_overlap_;
  hdmap::StopSignInfoConstPtr next_stop_sign_ = nullptr;
  double adc_distance_to_stop_sign_;
  std::vector<std::pair<
      hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>> associated_lanes_;

  // status related
  double stop_start_time_;
  StopSignLaneVehicles watch_vehicles_;

  // TODO(all): move to scenario conf later
  const uint32_t conf_start_stop_sign_timer = 10;  // second
  const double conf_min_pass_s_distance = 3.0;
  const double conf_max_adc_stop_speed = 0.3;
  const double conf_max_valid_stop_distance = 3.5;
  const double conf_stop_duration = 1.0;  // second
  const double conf_watch_vehicle_max_valid_stop_distance = 5.0;
};

}  // namespace planning
}  // namespace apollo
