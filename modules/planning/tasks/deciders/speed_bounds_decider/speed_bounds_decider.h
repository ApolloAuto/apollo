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

#pragma once

#include <memory>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class SpeedBoundsDecider : public Decider {
 public:
  SpeedBoundsDecider(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

  double SetSpeedFallbackDistance(PathDecision* const path_decision);

  void RecordSTGraphDebug(
      const StGraphData& st_graph_data,
      planning_internal::STGraphDebug* st_graph_debug) const;

 private:
  SpeedBoundsDeciderConfig speed_bounds_config_;
};

}  // namespace planning
}  // namespace apollo
