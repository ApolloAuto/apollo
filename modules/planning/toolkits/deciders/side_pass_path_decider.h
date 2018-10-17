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

#include "cybertron/common/macros.h"

#include "modules/planning/proto/decider_config.pb.h"

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/finite_element_qp/fem_1d_expanded_jerk_qp_problem.h"
#include "modules/planning/toolkits/deciders/decider.h"

namespace apollo {
namespace planning {

class SidePassPathDecider : public Decider {
 public:
  explicit SidePassPathDecider(const TaskConfig& config);

  enum SidePassDirection {
    LEFT = 0,
    RIGHT = 1,
  };

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  apollo::common::Status BuildSidePathDecision(
      Frame* frame, ReferenceLineInfo* const reference_line_info);

  bool GeneratePath(Frame* frame, ReferenceLineInfo* reference_line_info);

 private:
  std::unique_ptr<Fem1dQpProblem> fem_qp_;
  SidePassDirection decided_direction_;
  double delta_s_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
