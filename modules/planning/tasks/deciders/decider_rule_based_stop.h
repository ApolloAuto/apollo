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

#include <string>
#include <vector>

#include "modules/planning/proto/decider_config.pb.h"

#include "cyber/common/macros.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class DeciderRuleBasedStop : public Decider {
 public:
  explicit DeciderRuleBasedStop(const TaskConfig& config);

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  void CheckCrosswalk(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);

  void CheckStopSign(Frame* const frame,
                ReferenceLineInfo* const reference_line_info);

  void CheckTrafficLight(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  perception::TrafficLight ReadTrafficLight(
      const Frame& frame,
      const std::string& traffic_light_id);

  bool BuildStopDecision(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info,
                         const std::string& stop_wall_id,
                         const double stop_line_s,
                         const double stop_distance,
                         const StopReasonCode& stop_reason_code,
                         const std::vector<std::string>& wait_for_obstacles);

 private:
  static constexpr const char* STOP_SIGN_VO_ID_PREFIX = "SS_";
  static constexpr const char* TRAFFIC_LIGHT_VO_ID_PREFIX = "TL_";
};

}  // namespace planning
}  // namespace apollo
