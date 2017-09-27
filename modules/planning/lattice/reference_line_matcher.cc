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

#include "modules/planning/lattice/reference_line_matcher.h"

#include <algorithm>
#include <vector>
#include <cmath>

#include "modules/planning/math/hermite_spline.h"
#include "modules/planning/common/planning_util.h"
#include "modules/common/math/integral.h"
#include "modules/common/math/search.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::math::IntegrateByGaussLegendre;
using apollo::planning::util::InterpolateUsingLinearApproximation;
using apollo::common::math::GoldenSectionSearch;

PathPoint ReferenceLineMatcher::match_to_reference_line(
    const std::vector<PathPoint>& reference_line,
    const double x, const double y) {
  CHECK(reference_line.size() > 0);

  auto func_distance_square = [](const PathPoint& point,
      const double x, const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double distance_min = func_distance_square(reference_line.front(), x, y);
  std::size_t index_min = 0;

  for (std::size_t i = 1; i < reference_line.size(); ++i) {
    double distance_temp = func_distance_square(reference_line[i], x, y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = index_min == 0 ? index_min : index_min - 1;
  std::size_t index_end = index_min + 1 == reference_line.size() ?
      index_min : index_min + 1;

  if (index_start == index_end) {
    return reference_line[index_start];
  }

  return FindMinDistancePoint(reference_line[index_start],
                              reference_line[index_end], x, y);
}

PathPoint ReferenceLineMatcher::match_to_reference_line(
    const std::vector<PathPoint>& reference_line,
    const double s) {

  auto comp = [](const PathPoint& point, const double s) {
    return point.s() < s;
  };

  auto it_lower = std::lower_bound(reference_line.begin(),
                                   reference_line.end(), s, comp);
  if (it_lower == reference_line.begin()) {
    return reference_line.front();
  } else if (it_lower == reference_line.end()) {
    return reference_line.back();
  }

  // interpolate between it_lower - 1 and it_lower
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

PathPoint ReferenceLineMatcher::FindMinDistancePoint(
    const PathPoint& p0, const PathPoint& p1,
    const double x, const double y) {

  double heading_geodesic =
      apollo::common::math::NormalizeAngle(p1.theta() - p0.theta());
  HermiteSpline<double, 3> spline_geodesic({0.0, p0.kappa()},
      {heading_geodesic, p1.kappa()}, p0.s(), p1.s());

  auto func_dist_square = [&spline_geodesic, &p0, &x, &y](const double s) {
    auto func_cos_theta = [&spline_geodesic, &p0](const double s) {
      return std::cos(spline_geodesic.Evaluate(0, s) + p0.theta());
    };
    auto func_sin_theta = [&spline_geodesic, &p0](const double s) {
      return std::sin(spline_geodesic.Evaluate(0, s) + p0.theta());
    };
    double px = p0.x() + IntegrateByGaussLegendre(func_cos_theta, p0.s(), s);
    double py = p0.y() + IntegrateByGaussLegendre(func_sin_theta, p0.s(), s);

    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  double s = GoldenSectionSearch(func_dist_square, p0.s(), p1.s());
  return InterpolateUsingLinearApproximation(p0, p1, s);
}

} //namespace planning
} //namespace apollo
