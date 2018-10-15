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

#include "modules/planning/toolkits/deciders/side_pass_path_decider.h"

#include <string>
#include <tuple>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using ::apollo::common::ErrorCode;
using ::apollo::common::Status;
using ::apollo::hdmap::PathOverlap;

SidePassPathDecider::SidePassPathDecider(const TaskConfig &config)
    : Decider(config) {
  SetName("SidePassPathDecider");
  fem_qp_.reset(new Fem1dExpandedJerkQpProblem());
  // TODO(lianglia-apollo):
  // Put numbers into config when refactor is finished.
  const int n = 400;
  std::array<double, 3> l_init = {0.0, 0.0, 0.0};
  const double delta_s = 0.5;
  std::array<double, 5> w = {1.0, 2.0, 3.0, 4.0, 5.0};
  constexpr double kMaxLThirdOrderDerivative = 10.0;
  CHECK(fem_qp_->Init(n, l_init, delta_s, w, kMaxLThirdOrderDerivative));
}

Status SidePassPathDecider::Process(Frame *frame,
                                    ReferenceLineInfo *reference_line_info) {
  return Status::OK();
}

Status SidePassPathDecider::BuildSidePathDecision(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  // TODO(All): decide side pass from left or right.
  return Status().OK();
}

bool SidePassPathDecider::GeneratePath(Frame *frame,
                                       ReferenceLineInfo *reference_line_info) {
  // TODO(All): generate path here

  std::vector<std::tuple<double, double, double>> l_bounds;

  // TODO(All): set up l_bounds here.
  fem_qp_->SetVariableBounds(l_bounds);
  fem_qp_->Optimize();
  // TODO(All): put optimized results into ReferenceLineInfo.
  return true;
}

}  // namespace planning
}  // namespace apollo
