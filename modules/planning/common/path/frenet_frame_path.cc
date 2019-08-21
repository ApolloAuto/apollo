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
 * @file frenet_frame_path.cc
 **/
#include "modules/planning/common/path/frenet_frame_path.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {

using apollo::common::FrenetFramePoint;

FrenetFramePath::FrenetFramePath(std::vector<FrenetFramePoint> points)
    : std::vector<FrenetFramePoint>(std::move(points)) {}

double FrenetFramePath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

FrenetFramePoint FrenetFramePath::GetNearestPoint(const SLBoundary& sl) const {
  auto it_lower =
      std::lower_bound(begin(), end(), sl.start_s(), LowerBoundComparator);
  if (it_lower == end()) {
    return back();
  }
  auto it_upper =
      std::upper_bound(it_lower, end(), sl.end_s(), UpperBoundComparator);
  double min_dist = std::numeric_limits<double>::max();
  auto min_it = it_upper;
  for (auto it = it_lower; it != it_upper; ++it) {
    if (it->l() >= sl.start_l() && it->l() <= sl.end_l()) {
      return *it;
    } else if (it->l() > sl.end_l()) {
      double diff = it->l() - sl.end_l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    } else {
      double diff = sl.start_l() - it->l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    }
  }
  return *min_it;
}

FrenetFramePoint FrenetFramePath::EvaluateByS(const double s) const {
  CHECK_GT(size(), 1);
  auto it_lower = std::lower_bound(begin(), end(), s, LowerBoundComparator);
  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    return back();
  }
  const auto& p0 = *(it_lower - 1);
  const auto s0 = p0.s();
  const auto& p1 = *it_lower;
  const auto s1 = p1.s();

  FrenetFramePoint p;
  p.set_s(s);
  p.set_l(common::math::lerp(p0.l(), s0, p1.l(), s1, s));
  p.set_dl(common::math::lerp(p0.dl(), s0, p1.dl(), s1, s));
  p.set_ddl(common::math::lerp(p0.ddl(), s0, p1.ddl(), s1, s));
  return p;
}

}  // namespace planning
}  // namespace apollo
