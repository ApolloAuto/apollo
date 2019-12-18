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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_guide_line.h"

namespace apollo {
namespace planning {

void STGuideLine::Init(double desired_v) {
  s0_ = 0.0;
  t0_ = 0.0;
  v0_ = desired_v;
}

double STGuideLine::GetGuideSFromT(double t) const {
  return s0_ + (t - t0_) * v0_;
}

void STGuideLine::UpdateBlockingInfo(const double t, const double s_block,
                                     const bool is_lower_block) {
  if (is_lower_block) {
    if (GetGuideSFromT(t) < s_block) {
      s0_ = s_block;
      t0_ = t;
    }
  } else {
    if (GetGuideSFromT(t) > s_block) {
      s0_ = s_block;
      t0_ = t;
    }
  }
}

}  // namespace planning
}  // namespace apollo
