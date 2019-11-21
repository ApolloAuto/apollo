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
 * @file
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/planning/traffic_rules/traffic_rule.h"

namespace apollo {
namespace planning {

class Crosswalk : public TrafficRule {
 public:
  explicit Crosswalk(const TrafficRuleConfig& config);
  virtual ~Crosswalk() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool FindCrosswalks(ReferenceLineInfo* const reference_line_info);
  bool CheckStopForObstacle(ReferenceLineInfo* const reference_line_info,
                            const hdmap::CrosswalkInfoConstPtr crosswalk_ptr,
                            const Obstacle& obstacle,
                            const double stop_deceleration);

 private:
  static constexpr char const* CROSSWALK_VO_ID_PREFIX = "CW_";
  std::vector<const hdmap::PathOverlap*> crosswalk_overlaps_;
};

}  // namespace planning
}  // namespace apollo
