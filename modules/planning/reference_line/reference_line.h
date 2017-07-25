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
 * @file reference_line.h
 **/

#ifndef MODULES_PLANNING_MATH_REFERENCE_LINE_H_
#define MODULES_PLANNING_MATH_REFERENCE_LINE_H_

#include "Eigen/Dense"

#include "modules/common/proto/path_point.pb.h"

#include "modules/map/pnc_map/path.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  ReferenceLine(const std::vector<ReferencePoint>& reference_points,
                const std::vector<hdmap::LaneSegment>& lane_segments,
                const double max_approximation_error);
  void move(const ReferenceLine& reference_line);
  void move(const ReferenceLine&& reference_line);
  const hdmap::Path& reference_map_line() const;
  const std::vector<ReferencePoint>& reference_points() const;

  ReferencePoint get_reference_point(const double s) const;
  ReferencePoint get_reference_point(const double x, const double y) const;

  bool get_point_in_Cartesian_frame(const common::SLPoint& sl_point,
                                    Eigen::Vector2d* const xy_point) const;
  bool get_point_in_Frenet_frame(const Eigen::Vector2d& xy_point,
                                 common::SLPoint* const sl_point) const;

  double get_lane_width(const double s) const;
  bool is_on_road(const common::SLPoint& sl_point) const;

 private:
  static ReferencePoint interpolate(const ReferencePoint& p0, const double s0,
                                    const ReferencePoint& p1, const double s1,
                                    const double s);

  static double find_min_distance_point(const ReferencePoint& p0,
                                        const double s0,
                                        const ReferencePoint& p1,
                                        const double s1, const double x,
                                        const double y);

  std::vector<ReferencePoint> reference_points_;
  hdmap::Path reference_map_line_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_REFERENCE_LINE_H_
