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
 * @file reference_line.cc
 **/

#include "modules/planning/reference_line/reference_line.h"

#include "boost/math/tools/minima.hpp"

#include "modules/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/math/double.h"
//#include "modules/planning/math/interpolation.h"

namespace apollo {
namespace planning {

using ReferenceMapLine = ::apollo::hdmap::Path;

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : _reference_points(reference_points) {
  std::vector<::apollo::hdmap::PathPoint> points(reference_points.begin(),
                                                 reference_points.end());
  _reference_map_line = std::move(ReferenceMapLine(std::move(points)));
}

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points,
    const std::vector<::apollo::hdmap::LaneSegment>& lane_segments,
    const double max_approximation_error)
    : _reference_points(reference_points) {
  std::vector<::apollo::hdmap::PathPoint> points(reference_points.begin(),
                                                 reference_points.end());
  _reference_map_line = std::move(ReferenceMapLine(
      std::move(points), lane_segments, max_approximation_error));
}

void ReferenceLine::move(const ReferenceLine& reference_line) {
  _reference_map_line =
      ReferenceMapLine(reference_line.reference_map_line().path_points());
  _reference_points = reference_line.reference_points();
}

void ReferenceLine::move(const ReferenceLine&& reference_line) {
  _reference_map_line =
      ReferenceMapLine(reference_line.reference_map_line().path_points());
  _reference_points = reference_line.reference_points();
}

ReferencePoint ReferenceLine::get_reference_point(const double s) const {
  const auto& accumulated_s = _reference_map_line.accumulated_s();
  if (s < accumulated_s.front()) {
    AWARN << "The requested s is nearer than the start point of the reference "
             "line; reference line starts at "
          << accumulated_s.back() << ", requested " << s << ".";
    return _reference_points.front();
  }
  if (s > accumulated_s.back()) {
    AWARN
        << "The requested s exceeds the reference line; reference line ends at "
        << accumulated_s.back() << "requested " << s << " .";
    return _reference_points.back();
  }

  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  if (it_lower == accumulated_s.begin()) {
    return _reference_points.front();
  } else {
    std::size_t index = (std::size_t)(it_lower - accumulated_s.begin());
    auto p0 = _reference_points[index - 1];
    auto p1 = _reference_points[index];

    auto s0 = accumulated_s[index - 1];
    auto s1 = accumulated_s[index];

    return ReferenceLine::interpolate(p0, s0, p1, s1, s);
  }
}

double ReferenceLine::find_min_distance_point(const ReferencePoint& p0,
                                              const double s0,
                                              const ReferencePoint& p1,
                                              const double s1, const double x,
                                              const double y) {
  auto func_dist_square = [&p0, &p1, &s0, &s1, &x, &y](const double s) {
    auto p = ReferenceLine::interpolate(p0, s0, p1, s1, s);
    double dx = p.x() - x;
    double dy = p.y() - y;
    return dx * dx + dy * dy;
  };

  return ::boost::math::tools::brent_find_minima(func_dist_square, s0, s1, 8)
      .first;
}

ReferencePoint ReferenceLine::get_reference_point(const double x,
                                                  const double y) const {
  CHECK(_reference_points.size() > 0);

  auto func_distance_square = [](const ReferencePoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double d_min = func_distance_square(_reference_points.front(), x, y);
  double index_min = 0;

  for (size_t i = 1; i < _reference_points.size(); ++i) {
    double d_temp = func_distance_square(_reference_points[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == _reference_points.size() ? index_min : index_min + 1;

  if (index_start == index_end) return _reference_points[index_start];

  double s0 = _reference_map_line.accumulated_s()[index_start];
  double s1 = _reference_map_line.accumulated_s()[index_end];

  double s = ReferenceLine::find_min_distance_point(
      _reference_points[index_start], s0, _reference_points[index_end], s1, x,
      y);

  return ReferenceLine::interpolate(_reference_points[index_start], s0,
                                    _reference_points[index_end], s1, s);
}

bool ReferenceLine::get_point_in_Cartesian_frame(
    const apollo::common::SLPoint& sl_point,
    Eigen::Vector2d* const xy_point) const {
  CHECK_NOTNULL(xy_point);
  if (_reference_map_line.num_points() < 2) {
    AERROR << "The reference line has too few points.";
    return false;
  }

  const auto matched_point = get_reference_point(sl_point.s());
  const auto angle =
      ::apollo::common::math::Angle16::from_rad(matched_point.heading());
  (*xy_point)[0] =
      (matched_point.x() - ::apollo::common::math::sin(angle) * sl_point.l());
  (*xy_point)[1] =
      (matched_point.y() + ::apollo::common::math::cos(angle) * sl_point.l());
  return true;
}

bool ReferenceLine::get_point_in_Frenet_frame(
    const Eigen::Vector2d& xy_point,
    apollo::common::SLPoint* const sl_point) const {
  CHECK_NOTNULL(sl_point);
  double s = 0;
  double l = 0;
  if (!_reference_map_line.get_nearest_point(
          apollo::common::math::Vec2DCtor(xy_point.x(), xy_point.y()),
          &s, &l)) {
    AERROR << "Can't get nearest point from path.";
    return false;
  }

  if (s > _reference_map_line.accumulated_s().back()) {
    AERROR << "The s of point is bigger than the length of current path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

ReferencePoint ReferenceLine::interpolate(const ReferencePoint& p0,
                                          const double s0,
                                          const ReferencePoint& p1,
                                          const double s1, const double s) {
  ReferencePoint p = p1;
  p.set_x(apollo::common::math::lerp(p0.x(), s0, p1.x(), s1, s));
  p.set_y(apollo::common::math::lerp(p0.y(), s0, p1.y(), s1, s));
  p.set_heading(
      apollo::common::math::slerp(p0.heading(), s0, p1.heading(), s1, s));
  p.set_kappa(apollo::common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s));
  p.set_dkappa(apollo::common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s));

  // lane boundary info, lane info will be the same as the p1.
  return p;
}

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return _reference_points;
}

const ReferenceMapLine& ReferenceLine::reference_map_line() const {
  return _reference_map_line;
}

double ReferenceLine::get_lane_width(const double s) const {
  // TODO : need implement;
  return 4.0;
}

bool ReferenceLine::is_on_road(const apollo::common::SLPoint& sl_point) const {
  // TODO : need implement;
  return true;
}

}  // namespace planning
}  // namespace apollo
