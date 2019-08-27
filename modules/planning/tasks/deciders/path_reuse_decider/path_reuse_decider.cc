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
#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

namespace {
static int reusable_path_counter = 0;
}

PathReuseDecider::PathReuseDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  bool enable_path_reuse = true;

  // Check if path is reusable, if so, reuse previous path.
  if (enable_path_reuse && CheckPathReusable(frame)) {
    // count reusable path
    ++reusable_path_counter;
  }
  return Status::OK();
}

bool PathReuseDecider::CheckPathReusable(Frame* const frame) { return true; }

}  // namespace planning
}  // namespace apollo
