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
 * @file base_stage_traffic_light_creep.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/planning_interface_base/scenario_base/base_stage_creep.h"

namespace apollo {
namespace planning {

class BaseStageTrafficLightCreep : public BaseStageCreep {
 private:
  /**
   * @brief Get the overlap id of stage and the stop line distance according to
   * the frame and reference line information.
   *
   * @param frame current frame information
   * @param reference_line_info current reference line information
   * @param overlap_end_s end distance mapped to reference line of the overlap
   * @param overlap_id overlap id of current stage
   * @return return true if find a valid overlap
   */
  bool GetOverlapStopInfo(Frame* frame, ReferenceLineInfo* reference_line_info,
                          double* overlap_end_s,
                          std::string* overlap_id) const override;
};

}  // namespace planning
}  // namespace apollo
