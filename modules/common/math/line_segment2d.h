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
 * @file
 * @brief Define the LineSegment2d class.
 */

#pragma once

#include <string>

#include "modules/common/math/vec2d.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @class LineSegment2d
 * @brief Line segment in 2-D.
 */
class LineSegment2d {
 public:
  /**
   * @brief Empty constructor.
   */
  LineSegment2d();

  /**
   * @brief Constructor with start point and end point.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  LineSegment2d(const Vec2d &start, const Vec2d &end);

  /**
   * @brief Get the start point.
   * @return The start point of the line segment.
   */
  const Vec2d &start() const { return start_; }

  /**
   * @brief Get the end point.
   * @return The end point of the line segment.
   */
  const Vec2d &end() const { return end_; }

  /**
   * @brief Get the unit direction from the start point to the end point.
   * @return The start point of the line segment.
   */
  const Vec2d &unit_direction() const { return unit_direction_; }

  /**
   * @brief Get the center of the line segment.
   * @return The center of the line segment.
   */
  Vec2d center() const { return (start_ + end_) / 2.0; }

  /**
   * @brief Get the heading of the line segment.
   * @return The heading, which is the angle between unit direction and x-axis.
   */
  double heading() const { return heading_; }

  /**
   * @brief Get the cosine of the heading.
   * @return The cosine of the heading.
   */
  double cos_heading() const { return unit_direction_.x(); }

  /**
   * @brief Get the sine of the heading.
   * @return The sine of the heading.
   */
  double sin_heading() const { return unit_direction_.y(); }

  /**
   * @brief Get the length of the line segment.
   * @return The length of the line segment.
   */
  double length() const;

  /**
   * @brief Get the square of length of the line segment.
   * @return The square of length of the line segment.
   */
  double length_sqr() const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D.
   * @param point The point to compute the distance to.
   * @return The shortest distance from points on the line segment to point.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D, and get the nearest point on the line segment.
   * @param point The point to compute the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  double DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D.
   * @param point The point to compute the squared of the distance to.
   * @return The square of the shortest distance from points
   *         on the line segment to the input point.
   */
  double DistanceSquareTo(const Vec2d &point) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D,
   *        and get the nearest point on the line segment.
   * @param point The point to compute the squared of the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  double DistanceSquareTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Check if a point is within the line segment.
   * @param point The point to check if it is within the line segment.
   * @return Whether the input point is within the line segment or not.
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Check if the line segment has an intersect
   *        with another line segment in 2-D.
   * @param other_segment The line segment to check if it has an intersect.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool HasIntersect(const LineSegment2d &other_segment) const;

  /**
   * @brief Compute the intersect with another line segment in 2-D if any.
   * @param other_segment The line segment to compute the intersect.
   * @param point the computed intersect between the line segment and
   *        the input other_segment.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool GetIntersect(const LineSegment2d &other_segment,
                    Vec2d *const point) const;

  /**
   * @brief Compute the projection of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the projection onto the line segment.
   * @return The projection of the vector, which is from the start point of
   *         the line segment to the input point, onto the unit direction.
   */
  double ProjectOntoUnit(const Vec2d &point) const;

  /**
   * @brief Compute the cross product of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the cross product onto the line segment.
   * @return The cross product of the unit direction and
   *         the vector, which is from the start point of
   *         the line segment to the input point.
   */
  double ProductOntoUnit(const Vec2d &point) const;

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the straight line
   *        expanded from the line segment.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the straight line expanded from the line segment.
   * @return The distance from the input point to the perpendicular foot.
   */
  double GetPerpendicularFoot(const Vec2d &point,
                              Vec2d *const foot_point) const;

  /**
   * @brief Get the debug string including the essential information.
   * @return Information of the line segment for debugging.
   */
  std::string DebugString() const;

 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};

}  // namespace math
}  // namespace common
}  // namespace apollo
