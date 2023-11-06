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
 * @brief The class of Box2d. Here, the x/y axes are respectively Forward/Left,
 *        as opposed to what happens in euler_angles_zxy.h (Right/Forward).
 */

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @class Box2d
 * @brief Rectangular (undirected) bounding box in 2-D.
 *
 * This class is referential-agnostic, although our convention on the use of
 * the word "heading" in this project (permanently set to be 0 at East)
 * forces us to assume that the X/Y frame here is East/North.
 * For disambiguation, we call the axis of the rectangle parallel to the
 * heading direction the "heading-axis". The size of the heading-axis is
 * called "length", and the size of the axis perpendicular to it "width".
 */
class Box2d {
 public:
  Box2d() = default;
  /**
   * @brief Constructor which takes the center, heading, length and width.
   * @param center The center of the rectangular bounding box.
   * @param heading The angle between the x-axis and the heading-axis,
   *        measured counter-clockwise.
   * @param length The size of the heading-axis.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
  Box2d(const Vec2d &center, const double heading, const double length,
        const double width);

  /**
   * @brief Constructor which takes the point on the axis, front length, back
   * length, heading, and width.
   * @param center The center of the rectangular bounding box.
   * @param heading The angle between the x-axis and the heading-axis,
   *        measured counter-clockwise.
   * @param front_length The length from the start point to the given point.
   * @param back_length The length from the end point to the given point.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
  Box2d(const Vec2d &point, double heading, double front_length,
        double back_length, double width);

  /**
   * @brief Constructor which takes the heading-axis and the width of the box
   * @param axis The heading-axis
   * @param width The width of the box, which is taken perpendicularly
   * to the heading direction.
   */
  Box2d(const LineSegment2d &axis, const double width);

  /**
   * @brief Constructor which takes an AABox2d (axes-aligned box).
   * @param aabox The input AABox2d.
   */
  explicit Box2d(const AABox2d &aabox);

  /**
   * @brief Creates an axes-aligned Box2d from two opposite corners
   * @param one_corner One of the corners
   * @param opposite_corner The opposite corner to the first one
   * @return An axes-aligned Box2d
   */
  static Box2d CreateAABox(const Vec2d &one_corner,
                           const Vec2d &opposite_corner);

  /**
   * @brief Getter of the center of the box
   * @return The center of the box
   */
  const Vec2d &center() const { return center_; }

  /**
   * @brief Getter of the x-coordinate of the center of the box
   * @return The x-coordinate of the center of the box
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of the y-coordinate of the center of the box
   * @return The y-coordinate of the center of the box
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of the length
   * @return The length of the heading-axis
   */
  double length() const { return length_; }

  /**
   * @brief Getter of the width
   * @return The width of the box taken perpendicularly to the heading
   */
  double width() const { return width_; }

  /**
   * @brief Getter of half the length
   * @return Half the length of the heading-axis
   */
  double half_length() const { return half_length_; }

  /**
   * @brief Getter of half the width
   * @return Half the width of the box taken perpendicularly to the heading
   */
  double half_width() const { return half_width_; }

  /**
   * @brief Getter of the heading
   * @return The counter-clockwise angle between the x-axis and the heading-axis
   */
  double heading() const { return heading_; }

  /**
   * @brief Getter of the cosine of the heading
   * @return The cosine of the heading
   */
  double cos_heading() const { return cos_heading_; }

  /**
   * @brief Getter of the sine of the heading
   * @return The sine of the heading
   */
  double sin_heading() const { return sin_heading_; }

  /**
   * @brief Getter of the area of the box
   * @return The product of its length and width
   */
  double area() const { return length_ * width_; }

  /**
   * @brief Getter of the size of the diagonal of the box
   * @return The diagonal size of the box
   */
  double diagonal() const { return std::hypot(length_, width_); }

  /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
  const std::vector<Vec2d> &GetAllCorners() const;

  /**
   * @brief Tests points for membership in the box
   * @param point A point that we wish to test for membership in the box
   * @return True iff the point is contained in the box
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Tests points for membership in the boundary of the box
   * @param point A point that we wish to test for membership in the boundary
   * @return True iff the point is a boundary point of the box
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the box and a given point
   * @param point The point whose distance to the box we wish to compute
   * @return A distance
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the box and a given line segment
   * @param line_segment The line segment whose distance to the box we compute
   * @return A distance
   */
  double DistanceTo(const LineSegment2d &line_segment) const;

  /**
   * @brief Determines the distance between two boxes
   * @param box The box whose distance to this box we want to compute
   * @return A distance
   */
  double DistanceTo(const Box2d &box) const;

  /**
   * @brief Determines whether this box overlaps a given line segment
   * @param line_segment The line-segment
   * @return True if they overlap
   */
  bool HasOverlap(const LineSegment2d &line_segment) const;

  /**
   * @brief Determines whether these two boxes overlap
   * @param line_segment The other box
   * @return True if they overlap
   */
  bool HasOverlap(const Box2d &box) const;

  /**
   * @brief Gets the smallest axes-aligned box containing the current one
   * @return An axes-aligned box
   */
  AABox2d GetAABox() const;

  /**
   * @brief Rotate from center.
   * @param rotate_angle Angle to rotate.
   */
  void RotateFromCenter(const double rotate_angle);

  /**
   * @brief Shifts this box by a given vector
   * @param shift_vec The vector determining the shift
   */
  void Shift(const Vec2d &shift_vec);

  /**
   * @brief Extend the box longitudinally
   * @param extension_length the length to extend
   */
  void LongitudinalExtend(const double extension_length);

  void LateralExtend(const double extension_length);

  /**
   * @brief Gets a human-readable description of the box
   * @return A debug-string
   */
  std::string DebugString() const;

  void InitCorners();

  double max_x() const { return max_x_; }
  double min_x() const { return min_x_; }
  double max_y() const { return max_y_; }
  double min_y() const { return min_y_; }

 private:
  inline bool is_inside_rectangle(const Vec2d &point) const {
    return (point.x() >= 0.0 && point.x() <= width_ && point.y() >= 0.0 &&
            point.y() <= length_);
  }

  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;

  std::vector<Vec2d> corners_;

  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};

}  // namespace math
}  // namespace common
}  // namespace apollo
