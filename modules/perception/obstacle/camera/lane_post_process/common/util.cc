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

#include "modules/perception/obstacle/camera/lane_post_process/common/util.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace perception {

void RectAngle(ScalarType *theta) {
  if (theta == NULL) {
    return;
  }
  if (*theta < 0) {
    (*theta) += static_cast<ScalarType>(2 * M_PI);
  }
}

void NonMask::AddPolygonPoint(const ScalarType &x, const ScalarType &y) {
  polygon_.push_back(Vector2D(x, y));
}

int NonMask::ComputeOrientation(const Vector2D &p1, const Vector2D &p2,
                                const Vector2D &q) const {
  ScalarType cross = (q.x() - p1.x()) * (p2.y() - p1.y()) -
                     (p2.x() - p1.x()) * (q.y() - p1.y());
  if (std::fabs(cross) < kEpsilon) {
    return 0;  // coliner
  }
  return cross > 0 ? 1 : -1;  // 1: clockwise, -1: anti-clockwise
}

bool NonMask::IsColinear(const Vector2D &p1, const Vector2D &p2,
                         const Vector2D &q) const {
  ScalarType cross = (q.x() - p1.x()) * (p2.y() - p1.y()) -
                     (p2.x() - p1.x()) * (q.y() - p1.y());
  return std::fabs(cross) < kEpsilon;
}

bool NonMask::IsOnLineSegmentWhenColinear(const Vector2D &p1,
                                          const Vector2D &p2,
                                          const Vector2D &q) const {
  return q.x() <= std::max(p1.x(), p2.x()) &&
         q.x() >= std::min(p1.x(), p2.x()) &&
         q.y() <= std::max(p1.y(), p2.y()) && q.y() >= std::min(p1.y(), p2.y());
}

bool NonMask::IsLineSegmentIntersect(const Vector2D &p1, const Vector2D &p2,
                                     const Vector2D &p3,
                                     const Vector2D &p4) const {
  int relative_orientation_123 = ComputeOrientation(p1, p2, p3);
  int relative_orientation_124 = ComputeOrientation(p1, p2, p4);
  int relative_orientation_341 = ComputeOrientation(p3, p4, p1);
  int relative_orientation_342 = ComputeOrientation(p3, p4, p2);

  if (relative_orientation_123 == 0 &&
      IsOnLineSegmentWhenColinear(p1, p2, p3)) {
    return true;
  }
  if (relative_orientation_124 == 0 &&
      IsOnLineSegmentWhenColinear(p1, p2, p4)) {
    return true;
  }
  if (relative_orientation_341 == 0 &&
      IsOnLineSegmentWhenColinear(p3, p4, p1)) {
    return true;
  }
  if (relative_orientation_342 == 0 &&
      IsOnLineSegmentWhenColinear(p3, p4, p2)) {
    return true;
  }

  return (relative_orientation_123 != relative_orientation_124) &&
         (relative_orientation_341 != relative_orientation_342);
}

bool NonMask::IsInsideMask(const Vector2D &p) const {
  int n = static_cast<int>(polygon_.size());
  if (n < 3) return false;

  Vector2D extreme_p(ScalarType(INF_NON_MASK_POINT_X), p.y());

  int intersec_count = 0, curt = 0, next = 1;
  while (curt < n) {
    // Check if the line segment 'p'->'extreme_p' intersects with
    // 'polygon_[curt]'->'polygon_[next]'
    if (IsLineSegmentIntersect(polygon_[curt], polygon_[next], p, extreme_p)) {
      // If the point is colinear with the boundary polygon,
      // check if it lies on the boundary.
      if (IsColinear(polygon_[curt], polygon_[next], p)) {
        return IsOnLineSegmentWhenColinear(polygon_[curt], polygon_[next], p);
      }

      intersec_count++;
    }

    ++curt;
    if (++next == n) {
      next = 0;
    }
  }

  return intersec_count % 2 != 0;
}

}  // namespace perception
}  // namespace apollo
