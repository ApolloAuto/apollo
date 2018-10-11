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

#include "modules/planning/toolkits/deciders/side_pass_safety.h"

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using ::apollo::common::ErrorCode;
using ::apollo::common::Status;
using ::apollo::hdmap::PathOverlap;

bool SidePassSafety::Init(const ScenarioConfig::ScenarioTaskConfig &config) {
  is_init_ = true;
  return true;
}

Status SidePassSafety::Process(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR,
                  "SidePassSafety DeciderNot inited.");
  }

  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
