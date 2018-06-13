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
 * @brief This file provides the implementation of the class "NaviPathDecider".
 */

#include "modules/planning/navi/decider/navi_path_decider.h"

#include "glog/logging.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

NaviPathDecider::NaviPathDecider() : Task("NaviPathDecider") {
  // TODO(all): Add your other initialization.
}

Status NaviPathDecider::Execute(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = Process();
  RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

Status NaviPathDecider::Process() {
  // TODO(all): Add your concrete implementation code here. Use auxiliary member
  // functions when necessary.
  return Status::OK();
}

void NaviPathDecider::RecordDebugInfo(const PathData& path_data) {
  const auto& path_points = path_data.discretized_path().path_points();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

}  // namespace planning
}  // namespace apollo
