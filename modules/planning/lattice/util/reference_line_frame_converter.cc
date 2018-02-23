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

#include "modules/planning/lattice/util/reference_line_frame_converter.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"

namespace apollo {
namespace planning {

std::pair<double, double> ReferenceLineFrameConverter::CartesianToFrenet(
    const std::vector<common::PathPoint>& discretized_reference_line,
    const double x, const double y) {
  auto matched_point = ReferenceLineMatcher::MatchToReferenceLine(
      discretized_reference_line, x, y);

  double s = 0.0;
  double d = 0.0;

  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), x, y, &s, &d);

  return std::pair<double, double>(s, d);
}

}  // namespace planning
}  // namespace apollo
