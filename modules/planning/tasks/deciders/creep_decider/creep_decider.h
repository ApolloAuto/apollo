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

#include "cyber/common/macros.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class CreepDecider : public Decider {
 public:
  CreepDecider(const TaskConfig& config,
               const std::shared_ptr<DependencyInjector>& injector);

  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool CheckCreepDone(const Frame& frame,
                      const ReferenceLineInfo& reference_line_info,
                      const double stop_sign_overlap_end_s,
                      const double wait_time_sec, const double timeout_sec);

  double FindCreepDistance(const Frame& frame,
                           const ReferenceLineInfo& reference_line_info);

 private:
  static constexpr const char* CREEP_VO_ID_PREFIX = "CREEP_";
  common::TrajectoryPoint adc_planning_start_point_;
  hdmap::Lane curr_lane_;
};

}  // namespace planning
}  // namespace apollo
