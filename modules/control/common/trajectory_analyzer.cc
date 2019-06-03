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

#include "modules/control/common/trajectory_analyzer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/search.h"
#include "modules/control/common/control_gflags.h"

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

namespace apollo {
namespace control {
namespace {

// Squared distance from the point to (x, y).
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  if (point.has_path_point()) {
    return point.path_point();
  } else {
    return PathPoint();
  }
}

}  // namespace

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const planning::ADCTrajectory *planning_published_trajectory) {
  header_time_ = planning_published_trajectory->header().timestamp_sec();
  seq_num_ = planning_published_trajectory->header().sequence_num();

  for (int i = 0; i < planning_published_trajectory->trajectory_point_size();
       ++i) {
    trajectory_points_.push_back(
        planning_published_trajectory->trajectory_point(i));
  }
}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                                    const double y) const {
  CHECK_GT(trajectory_points_.size(), 0);

  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  const double kEpsilon = 0.001;
  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].path_point().s() -
                trajectory_points_[index_end].path_point().s()) <= kEpsilon) {
    return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
  }

  return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y);
}

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenét Frame,
// Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.
void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  double dx = x - ref_point.x();
  double dy = y - ref_point.y();

  double cos_ref_theta = std::cos(ref_point.theta());
  double sin_ref_theta = std::sin(ref_point.theta());

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s() + dot_rd_nd;

  double delta_theta = theta - ref_point.theta();
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;

  double one_minus_kappa_r_d = 1 - ref_point.kappa() * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    AERROR << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa()
           << " ref_point.x:" << ref_point.x()
           << " ref_point.y:" << ref_point.y() << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const {
  return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const {
  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    return point.relative_time() < relative_time;
  };

  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);

  if (it_low == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }

  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  if (FLAGS_query_forward_time_point_only) {
    return *it_low;
  } else {
    auto it_lower = it_low - 1;
    if (it_low->relative_time() - t < t - it_lower->relative_time()) {
      return *it_low;
    }
    return *it_lower;
  }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) const {
  CHECK_GT(trajectory_points_.size(), 0);

  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return trajectory_points_[index_min];
}

const std::vector<TrajectoryPoint> &TrajectoryAnalyzer::trajectory_points()
    const {
  return trajectory_points_;
}

PathPoint TrajectoryAnalyzer::FindMinDistancePoint(const TrajectoryPoint &p0,
                                                   const TrajectoryPoint &p1,
                                                   const double x,
                                                   const double y) const {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  auto dist_square = [&p0, &p1, &x, &y](const double s) {
    double px = common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                                   p1.path_point().x(), p1.path_point().s(), s);
    double py = common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                                   p1.path_point().y(), p1.path_point().s(), s);
    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  PathPoint p = p0.path_point();
  double s = common::math::GoldenSectionSearch(dist_square, p0.path_point().s(),
                                               p1.path_point().s());
  p.set_s(s);
  p.set_x(common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                             p1.path_point().x(), p1.path_point().s(), s));
  p.set_y(common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                             p1.path_point().y(), p1.path_point().s(), s));
  p.set_theta(common::math::slerp(p0.path_point().theta(), p0.path_point().s(),
                                  p1.path_point().theta(), p1.path_point().s(),
                                  s));
  // approximate the curvature at the intermediate point
  p.set_kappa(common::math::lerp(p0.path_point().kappa(), p0.path_point().s(),
                                 p1.path_point().kappa(), p1.path_point().s(),
                                 s));
  return p;
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
  CHECK_GT(trajectory_points_.size(), 0);
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point());
    trajectory_points_[i].mutable_path_point()->set_x(com.x());
    trajectory_points_[i].mutable_path_point()->set_y(com.y());
  }
}

common::math::Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta());
  const double sin_heading = std::sin(path_point.theta());
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x(), path_point.y(), path_point.z());
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return common::math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace control
}  // namespace apollo
