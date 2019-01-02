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

#include "cyber/common/macros.h"

#include "modules/planning/proto/decider_config.pb.h"

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class SidePassSafety : public Decider {
 public:
  explicit SidePassSafety(const TaskConfig& config);

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  apollo::common::Status BuildSidePathDecision(
      Frame* frame, ReferenceLineInfo* reference_line_info);

  bool IsSafeSidePass(Frame* frame,
                      ReferenceLineInfo* const reference_line_info);

 private:
  static constexpr char const* const SIDEPASS_VIRTUAL_OBSTACLE_ID =
      "SP_side_pass_safety";
};

}  // namespace planning
}  // namespace apollo
