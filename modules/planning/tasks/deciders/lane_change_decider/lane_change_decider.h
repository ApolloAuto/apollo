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

#pragma once

#include <list>
#include <string>

#include "modules/planning/proto/planning_status.pb.h"

#include "modules/map/pnc_map/route_segments.h"
#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class LaneChangeDecider : public Decider {
 public:
  explicit LaneChangeDecider(const TaskConfig& config);

   /**
   * @brief static function to check if the ChangeLanePath type of reference
   * line is safe or if current reference line is safe to deviate away and come
   * back
   */
  static bool IsClearToChangeLane(ReferenceLineInfo* reference_line_info);

 private:
  common::Status Process(Frame* frame) override;

  static bool HysteresisFilter(const double obstacle_distance,
                               const double safe_distance,
                               const double distance_buffer,
                               const bool is_obstacle_blocking);

  void UpdateStatus(ChangeLaneStatus::Status status_code,
                    const std::string& path_id);
  void UpdateStatus(double timestamp, ChangeLaneStatus::Status status_code,
                    const std::string& path_id);

  void PrioritizeChangeLane(
      std::list<ReferenceLineInfo>* reference_line_info) const;

  void RemoveChangeLane(
      std::list<ReferenceLineInfo>* reference_line_info) const;

  std::string GetCurrentPathId(
      const std::list<ReferenceLineInfo>& reference_line_info) const;
};

}  // namespace planning
}  // namespace apollo
