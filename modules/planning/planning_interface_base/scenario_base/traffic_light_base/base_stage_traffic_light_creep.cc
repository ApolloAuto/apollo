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
 * @file base_stage_traffic_light_creep.cc
 **/

#include "modules/planning/planning_interface_base/scenario_base/traffic_light_base/base_stage_traffic_light_creep.h"

#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace planning {

bool BaseStageTrafficLightCreep::GetOverlapStopInfo(
    Frame* frame, ReferenceLineInfo* reference_line_info, double* overlap_end_s,
    std::string* overlap_id) const {
  std::string current_traffic_light_overlap_id;
  if (injector_->planning_context()
          ->planning_status()
          .traffic_light()
          .current_traffic_light_overlap_id_size() > 0) {
    current_traffic_light_overlap_id = injector_->planning_context()
                                           ->planning_status()
                                           .traffic_light()
                                           .current_traffic_light_overlap_id(0);
  }

  if (!current_traffic_light_overlap_id.empty()) {
    // get overlap along reference line
    hdmap::PathOverlap* current_traffic_light_overlap =
        reference_line_info->GetOverlapOnReferenceLine(
            current_traffic_light_overlap_id, ReferenceLineInfo::SIGNAL);
    if (current_traffic_light_overlap) {
      *overlap_end_s = current_traffic_light_overlap->end_s;
      *overlap_id = current_traffic_light_overlap_id;
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
