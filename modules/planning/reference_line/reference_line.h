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

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/map/proto/map.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  template <typename Iterator>
  ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  explicit ReferenceLine(const hdmap::Path& hdmap_path);

  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const ReferenceLine& other);

  bool Segment(const common::math::Vec2d& point, const double distance_backward,
               const double distance_forward);

  bool Segment(const double s, const double distance_backward,
               const double distance_forward);

  const hdmap::Path& map_path() const;
  const std::vector<ReferencePoint>& reference_points() const;

  ReferencePoint GetReferencePoint(const double s) const;

  common::FrenetFramePoint GetFrenetPoint(
      const common::PathPoint& path_point) const;

  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(
      const common::TrajectoryPoint& traj_point) const;

  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d& xy) const;

  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
                                                  const double end_s) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  ReferencePoint GetReferencePoint(const double x, const double y) const;

  bool GetApproximateSLBoundary(const common::math::Box2d& box,
                                const double start_s, const double end_s,
                                SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const common::math::Box2d& box,
                     SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const hdmap::Polygon& polygon,
                     SLBoundary* const sl_boundary) const;

  bool SLToXY(const common::SLPoint& sl_point,
              common::math::Vec2d* const xy_point) const;
  bool XYToSL(const common::math::Vec2d& xy_point,
              common::SLPoint* const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, common::SLPoint* const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
  }

  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;

  bool GetOffsetToMap(const double s, double* l_offset) const;

  bool GetRoadWidth(const double s, double* const road_left_width,
                    double* const road_right_width) const;

  hdmap::Road::Type GetRoadType(const double s) const;

  void GetLaneFromS(const double s,
                    std::vector<hdmap::LaneInfoConstPtr>* lanes) const;

  double GetDrivingWidth(const SLBoundary& sl_boundary) const;

  /**
   * @brief: check if a box/point is on lane along reference line
   */
  bool IsOnLane(const common::SLPoint& sl_point) const;
  bool IsOnLane(const common::math::Vec2d& vec2d_point) const;
  template <class XYPoint>
  bool IsOnLane(const XYPoint& xy) const {
    return IsOnLane(common::math::Vec2d(xy.x(), xy.y()));
  }
  bool IsOnLane(const SLBoundary& sl_boundary) const;

  /**
   * @brief: check if a box/point is on road
   *         (not on sideways/medians) along reference line
   */
  bool IsOnRoad(const common::SLPoint& sl_point) const;
  bool IsOnRoad(const common::math::Vec2d& vec2d_point) const;
  bool IsOnRoad(const SLBoundary& sl_boundary) const;

  /**
   * @brief Check if a box is blocking the road surface. The criteria is to
   * check whether the remaining space on the road surface is larger than the
   * provided gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */
  bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;

  /**
   * @brief check if any part of the box has overlap with the road.
   */
  bool HasOverlap(const common::math::Box2d& box) const;

  double Length() const { return map_path_.length(); }

  std::string DebugString() const;

  double GetSpeedLimitFromS(const double s) const;

  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

  uint32_t GetPriority() const { return priority_; }

  void SetPriority(uint32_t priority) { priority_ = priority; }

  const hdmap::Path& GetMapPath() const { return map_path_; }

 private:
  /**
   * @brief Linearly interpolate p0 and p1 by s0 and s1.
   * The input has to satisfy condition: s0 <= s <= s1
   * p0 and p1 must have lane_waypoint.
   * Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
   * parallel neighboring lanes. Otherwise the interpolated result may not
   * valid.
   * @param p0 the first anchor point for interpolation.
   * @param s0 the longitutial distance (s) of p0 on current reference line.
   * s0 <= s && s0 <= s1
   * @param p1 the second anchor point for interpolation
   * @param s1 the longitutial distance (s) of p1 on current reference line.
   * s1
   * @param s identifies the middle point that is going to be
   * interpolated.
   * s >= s0 && s <= s1
   * @return The interpolated ReferencePoint.
   */
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0,
                                    const ReferencePoint& p1, const double s1,
                                    const double s);
  ReferencePoint InterpolateWithMatchedIndex(
      const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
      const double s1, const hdmap::InterpolatedIndex& index) const;

  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  hdmap::Path map_path_;
  uint32_t priority_ = 0;
};

}  // namespace planning
}  // namespace apollo
