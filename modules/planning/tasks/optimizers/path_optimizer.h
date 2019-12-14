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
 * @file path_optimizer.h
 **/

#pragma once

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class PathOptimizer : public Task {
 public:
  explicit PathOptimizer(const TaskConfig &config);
  virtual ~PathOptimizer() = default;
  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 protected:
  virtual apollo::common::Status Process(
      const SpeedData &speed_data, const ReferenceLine &reference_line,
      const common::TrajectoryPoint &init_point, const bool path_reusable,
      PathData *const path_data) = 0;

  void RecordDebugInfo(const PathData &path_data);
};

}  // namespace planning
}  // namespace apollo
