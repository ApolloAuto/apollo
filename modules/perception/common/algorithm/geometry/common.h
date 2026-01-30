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

#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/box.h"
#include "modules/perception/common/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace algorithm {

// @brief check a point is in polygon or not
// old name: is_xy_point_in_2d_xy_polygon
template <typename PointT>
bool IsPointXYInPolygon2DXY(const PointT &point,
                            const base::PointCloud<PointT> &polygon) {
  using Type = typename PointT::Type;
  bool in_poly = false;
  Type x1 = 0.0;
  Type x2 = 0.0;
  Type y1 = 0.0;
  Type y2 = 0.0;
  size_t nr_poly_points = polygon.size();
  if (nr_poly_points < 3) {
    AINFO << "Polygon points number is smaller than 3.";
    return false;
  }

  // start with the last point to make the check last point<->first point the
  // first one
  Type xold = polygon.at(nr_poly_points - 1).x;
  Type yold = polygon.at(nr_poly_points - 1).y;
  for (size_t i = 0; i < nr_poly_points; ++i) {
    Type xnew = polygon.at(i).x;
    Type ynew = polygon.at(i).y;
    if (xnew > xold) {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    } else {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }
    // if the point is on the boundary, then it is defined as in the polygon
    Type value = (point.y - y1) * (x2 - x1) - (y2 - y1) * (point.x - x1);
    Type temp = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    if (temp < std::numeric_limits<Type>::epsilon()) {
      continue;
    }
    Type distance = std::abs(value) / temp;
    if (x1 <= point.x && point.x <= x2 &&
        distance < std::numeric_limits<Type>::epsilon()) {
      return true;
    }
    if ((x1 < point.x) == (point.x <= x2) && value < 0.f) {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }
  return in_poly;
}

// @brief check a point is in bounding-box or not
// old name: is_point_in_boundingbox
template <typename PointT>
bool IsPointInBBox(const Eigen::Matrix<typename PointT::Type, 3, 1> &gnd_c,
                   const Eigen::Matrix<typename PointT::Type, 3, 1> &dir_x,
                   const Eigen::Matrix<typename PointT::Type, 3, 1> &dir_y,
                   const Eigen::Matrix<typename PointT::Type, 3, 1> &dir_z,
                   const Eigen::Matrix<typename PointT::Type, 3, 1> &size,
                   const PointT &point) {
  using T = typename PointT::Type;
  Eigen::Matrix<T, 3, 1> eig(point.x, point.y, point.z);
  Eigen::Matrix<T, 3, 1> diff = eig - gnd_c;
  T x = diff.dot(dir_x);
  if (fabs(x) > size[0] * 0.5) {
    return false;
  }
  T y = diff.dot(dir_y);
  if (fabs(y) > size[1] * 0.5) {
    return false;
  }
  T z = diff.dot(dir_z);
  if (fabs(z) > size[2] * 0.5) {
    return false;
  }
  return true;
}

// @brief calculate the size and center of the bounding-box of a point cloud
// old name: compute_bbox_size_center_xy
template <typename PointCloudT>
void CalculateBBoxSizeCenter2DXY(
    const PointCloudT &cloud, const Eigen::Vector3f &dir, Eigen::Vector3f *size,
    Eigen::Vector3d *center,
    float minimum_edge_length = std::numeric_limits<float>::epsilon()) {
  // NOTE: direction should not be (0, 0, 1)
  Eigen::Matrix3d projection;
  Eigen::Vector3d dird(dir[0], dir[1], 0.0);
  dird.normalize();
  projection << dird[0], dird[1], 0.0, -dird[1], dird[0], 0.0, 0.0, 0.0, 1.0;
  constexpr double kDoubleMax = std::numeric_limits<double>::max();
  Eigen::Vector3d min_pt(kDoubleMax, kDoubleMax, kDoubleMax);
  Eigen::Vector3d max_pt(-kDoubleMax, -kDoubleMax, -kDoubleMax);
  Eigen::Vector3d loc_pt(0.0, 0.0, 0.0);
  for (size_t i = 0; i < cloud.size(); i++) {
    loc_pt = projection * Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z);

    min_pt(0) = std::min(min_pt(0), loc_pt(0));
    min_pt(1) = std::min(min_pt(1), loc_pt(1));
    min_pt(2) = std::min(min_pt(2), loc_pt(2));

    max_pt(0) = std::max(max_pt(0), loc_pt(0));
    max_pt(1) = std::max(max_pt(1), loc_pt(1));
    max_pt(2) = std::max(max_pt(2), loc_pt(2));
  }
  (*size) = (max_pt - min_pt).cast<float>();
  Eigen::Vector3d coeff = (max_pt + min_pt) * 0.5;
  coeff(2) = min_pt(2);
  *center = projection.transpose() * coeff;

  constexpr float kFloatEpsilon = std::numeric_limits<float>::epsilon();
  float minimum_size = std::max(minimum_edge_length, kFloatEpsilon);

  (*size)(0) = (*size)(0) <= minimum_size ? minimum_size : (*size)(0);
  (*size)(1) = (*size)(1) <= minimum_size ? minimum_size : (*size)(1);
  (*size)(2) = (*size)(2) <= minimum_size ? minimum_size : (*size)(2);
}

// old name: compute_most_consistent_bbox_direction
template <typename Type>
void CalculateMostConsistentBBoxDir2DXY(
    const Eigen::Matrix<Type, 3, 1> &prev_dir,
    Eigen::Matrix<Type, 3, 1> *curr_dir) {
  Type dot_val_00 = prev_dir(0) * (*curr_dir)(0) + prev_dir(1) * (*curr_dir)(1);
  Type dot_val_01 = prev_dir(0) * (*curr_dir)(1) - prev_dir(1) * (*curr_dir)(0);
  if (fabs(dot_val_00) >= fabs(dot_val_01)) {
    if (dot_val_00 < 0) {
      (*curr_dir) = -(*curr_dir);
    }
  } else {
    if (dot_val_01 < 0) {
      (*curr_dir) =
          Eigen::Matrix<Type, 3, 1>((*curr_dir)(1), -(*curr_dir)(0), 0);
    } else {
      (*curr_dir) =
          Eigen::Matrix<Type, 3, 1>(-(*curr_dir)(1), (*curr_dir)(0), 0);
    }
  }
}

// @brief calculate the IOU (intersection-over-union) between two bbox
// old name:compute_2d_iou_bbox_to_bbox
template <typename Type>
Type CalculateIou2DXY(const Eigen::Matrix<Type, 3, 1> &center0,
                      const Eigen::Matrix<Type, 3, 1> &size0,
                      const Eigen::Matrix<Type, 3, 1> &center1,
                      const Eigen::Matrix<Type, 3, 1> &size1) {
  Type min_x_bbox_0 = center0(0) - size0(0) * static_cast<Type>(0.5);
  Type min_x_bbox_1 = center1(0) - size1(0) * static_cast<Type>(0.5);
  Type max_x_bbox_0 = center0(0) + size0(0) * static_cast<Type>(0.5);
  Type max_x_bbox_1 = center1(0) + size1(0) * static_cast<Type>(0.5);
  Type start_x = std::max(min_x_bbox_0, min_x_bbox_1);
  Type end_x = std::min(max_x_bbox_0, max_x_bbox_1);
  Type length_x = end_x - start_x;
  if (length_x <= 0) {
    return 0;
  }
  Type min_y_bbox_0 = center0(1) - size0(1) * static_cast<Type>(0.5);
  Type min_y_bbox_1 = center1(1) - size1(1) * static_cast<Type>(0.5);
  Type max_y_bbox_0 = center0(1) + size0(1) * static_cast<Type>(0.5);
  Type max_y_bbox_1 = center1(1) + size1(1) * static_cast<Type>(0.5);
  Type start_y = std::max(min_y_bbox_0, min_y_bbox_1);
  Type end_y = std::min(max_y_bbox_0, max_y_bbox_1);
  Type length_y = end_y - start_y;
  if (length_y <= 0) {
    return 0;
  }
  Type intersection_area = length_x * length_y;
  Type bbox_0_area = size0(0) * size0(1);
  Type bbox_1_area = size1(0) * size1(1);
  Type iou =
      intersection_area / (bbox_0_area + bbox_1_area - intersection_area);
  return iou;
}
template <typename Type>
Type CalculateIOUBBox(const base::BBox2D<Type> &box1,
                      const base::BBox2D<Type> &box2) {
  base::Rect<Type> rect1(box1);
  base::Rect<Type> rect2(box2);
  base::Rect<Type> intersection = rect1 & rect2;
  base::Rect<Type> unionsection = rect1 | rect2;
  return intersection.Area() / unionsection.Area();
}

// @brief given a point and segments,
// calculate the distance and direction to the nearest segment
// old name: calculate_distance_and_direction_to_segments_xy
template <typename PointT>
bool CalculateDistAndDirToSegs(
    const Eigen::Matrix<typename PointT::Type, 3, 1> &pt,
    const base::PointCloud<PointT> &segs, typename PointT::Type *dist,
    Eigen::Matrix<typename PointT::Type, 3, 1> *dir) {
  if (segs.size() < 2) {
    return false;
  }

  using Type = typename PointT::Type;
  Eigen::Matrix<Type, 3, 1> seg_point(segs[0].x, segs[0].y, 0);
  Type min_dist = (pt - seg_point).head(2).norm();

  Eigen::Matrix<Type, 3, 1> end_point_pre;
  Eigen::Matrix<Type, 3, 1> end_point_cur;
  Eigen::Matrix<Type, 3, 1> line_segment_dir;
  Eigen::Matrix<Type, 3, 1> line_segment_dir_pre;
  Eigen::Matrix<Type, 3, 1> end_point_to_pt_vec;

  line_segment_dir_pre << 0, 0, 0;

  Type line_segment_len = 0;
  Type projected_len = 0;
  Type point_to_line_dist = 0;
  Type point_to_end_point_dist = 0;

  for (size_t i = 1; i < segs.size(); ++i) {
    end_point_pre << segs[i - 1].x, segs[i - 1].y, 0;
    end_point_cur << segs[i].x, segs[i].y, 0;
    line_segment_dir = end_point_pre - end_point_cur;
    end_point_to_pt_vec = pt - end_point_cur;
    end_point_to_pt_vec(2) = 0;
    line_segment_len = line_segment_dir.head(2).norm();
    line_segment_dir = line_segment_dir / line_segment_len;
    if (i == 1) {
      *dir = line_segment_dir;
    }
    projected_len = end_point_to_pt_vec.dot(line_segment_dir);
    // case 1. pt is in the range of current line segment, compute
    // the point to line distance
    if (projected_len >= 0 && projected_len <= line_segment_len) {
      point_to_line_dist = end_point_to_pt_vec.cross(line_segment_dir).norm();
      if (min_dist > point_to_line_dist) {
        min_dist = point_to_line_dist;
        *dir = line_segment_dir;
      }
    } else {
      // case 2. pt is out of range of current line segment, compute
      // the point to end point distance
      point_to_end_point_dist = end_point_to_pt_vec.head(2).norm();
      if (min_dist > point_to_end_point_dist) {
        min_dist = point_to_end_point_dist;
        *dir = line_segment_dir + line_segment_dir_pre;
        dir->normalize();
      }
    }
    line_segment_dir_pre = line_segment_dir;
  }
  *dist = min_dist;

  return true;
}

// @brief given a point and two boundaries,
// calculate the distance and direction to the nearer boundary
// old name: calculate_distance_and_direction_to_boundary_xy
template <typename PointT>
void CalculateDistAndDirToBoundary(
    const Eigen::Matrix<typename PointT::Type, 3, 1> &pt,
    const base::PointCloud<PointT> &left_boundary,
    const base::PointCloud<PointT> &right_boundary, typename PointT::Type *dist,
    Eigen::Matrix<typename PointT::Type, 3, 1> *dir) {
  using Type = typename PointT::Type;
  Type dist_to_left = std::numeric_limits<Type>::max();
  Eigen::Matrix<Type, 3, 1> direction_left;
  Type dist_to_right = std::numeric_limits<Type>::max();
  Eigen::Matrix<Type, 3, 1> direction_right;

  CalculateDistAndDirToSegs(pt, left_boundary, &dist_to_left, &direction_left);

  CalculateDistAndDirToSegs(pt, right_boundary, &dist_to_right,
                            &direction_right);

  if (dist_to_left < dist_to_right) {
    (*dist) = dist_to_left;
    (*dir) = direction_left;
  } else {
    (*dist) = dist_to_right;
    (*dir) = direction_right;
  }
}

// @brief given a point and two boundaries sets,
// calculate the distance and direction to the nearest boundary
// old name: calculate_distance_and_direction_to_boundary_xy
template <typename PointT>
void CalculateDistAndDirToBoundary(
    const Eigen::Matrix<typename PointT::Type, 3, 1> &pt,
    const apollo::common::EigenVector<base::PointCloud<PointT>> &left_boundary,
    const apollo::common::EigenVector<base::PointCloud<PointT>> &right_boundary,
    typename PointT::Type *dist,
    Eigen::Matrix<typename PointT::Type, 3, 1> *dir) {
  using Type = typename PointT::Type;
  Type dist_to_left = std::numeric_limits<Type>::max();
  Eigen::Matrix<Type, 3, 1> direction_left;
  Type dist_to_right = std::numeric_limits<Type>::max();
  Eigen::Matrix<Type, 3, 1> direction_right;

  for (size_t i = 0; i < left_boundary.size(); i++) {
    Type dist_temp = std::numeric_limits<Type>::max();
    Eigen::Matrix<Type, 3, 1> dir_temp;
    if (CalculateDistAndDirToSegs(pt, left_boundary[i], &dist_temp,
                                  &dir_temp)) {
      if (dist_to_left > dist_temp) {
        dist_to_left = dist_temp;
        direction_left = dir_temp;
      }
    }
  }

  for (size_t i = 0; i < right_boundary.size(); i++) {
    Type dist_temp = std::numeric_limits<Type>::max();
    Eigen::Matrix<Type, 3, 1> dir_temp;
    if (CalculateDistAndDirToSegs(pt, right_boundary[i], &dist_temp,
                                  &dir_temp)) {
      if (dist_to_right > dist_temp) {
        dist_to_right = dist_temp;
        direction_right = dir_temp;
      }
    }
  }
  if (dist_to_left < dist_to_right) {
    (*dist) = dist_to_left;
    (*dir) = direction_left;
  } else {
    (*dist) = dist_to_right;
    (*dir) = direction_right;
  }
}

// @brief given center size and dir
// calculate the other-four corners
template <typename Type>
void CalculateCornersFromCenter(Type center_x, Type center_y, Type center_z,
    Type size_x, Type size_y, Type size_z, Type theta,
    Eigen::Matrix<Type, 8, 1> *corners) {
    Type cos_theta = cos(theta);
    Type sin_theta = sin(theta);
    Type hx = size_x * 0.5;
    Type hy = size_y * 0.5;

    Type left_up_x = (-hx) * cos_theta + (-hy) * sin_theta + center_x;
    Type left_up_y = (-hx) * (-sin_theta) + (-hy) * cos_theta + center_y;
    Type right_up_x = (-hx) * cos_theta + hy * sin_theta + center_x;
    Type right_up_y = (-hx) * (-sin_theta) + hy * cos_theta + center_y;
    Type right_down_x = hx * cos_theta + hy * sin_theta + center_x;
    Type right_down_y = hx * (-sin_theta) + hy * cos_theta + center_y;
    Type left_down_x = hx * cos_theta + (-hy) * sin_theta + center_x;
    Type left_down_y = hx * (-sin_theta) + (-hy) * cos_theta + center_y;
    
    (*corners)(0) = left_up_x;
    (*corners)(1) = left_up_y;
    (*corners)(2) = right_up_x;
    (*corners)(3) = right_up_y;
    (*corners)(4) = right_down_x;
    (*corners)(5) = right_down_y;
    (*corners)(6) = left_down_x;
    (*corners)(7) = left_down_y;
}

// @brief return true if two 2D line segment `(p1, q1)` and
// `(p2, q2)` intersect
template <typename T>
bool IsLineSegments2DIntersect(
    const Eigen::Matrix<T, 2, 1> &p1,
    const Eigen::Matrix<T, 2, 1> &q1,
    const Eigen::Matrix<T, 2, 1> &p2,
    const Eigen::Matrix<T, 2, 1> &q2) {
  // find orientation of ordered triplet (p, q, r).
  // returns values:
  // 0 --> p, q and r are collinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  auto find_orientation = [](const Eigen::Matrix<T, 2, 1> &p,
                              const Eigen::Matrix<T, 2, 1> &q,
                              const Eigen::Matrix<T, 2, 1> &r) -> int {
    // comparing slopes of line segments (p, q) and (q, r)
    // k1 = (q[1] - p[1]) / (q[0] - p[0])
    // k2 = (r[1] - q[1]) / (r[0] - q[0])
    // if k1 < k2, -> counterclockwise
    // if k1 > k2, -> clockwise
    // if k1 == k2, -> collinear
    float ret = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    // collinear
    if (std::fabs(ret) < std::numeric_limits<float>::epsilon()) {
      return 0;
    }
    // clock or counterclock wise
    return (ret > 0.f) ? 1 : 2;
  };

  // given three 'collinear' points p, q, r
  // check whether point p is on line segment (q, r)
  auto is_on_segment = [](const Eigen::Matrix<T, 2, 1> &p,
                          const Eigen::Matrix<T, 2, 1> &q,
                          const Eigen::Matrix<T, 2, 1> &r) -> bool {
    return p[0] <= std::max(q[0], r[0])
          && p[0] >= std::min(q[0], r[0])
          && p[1] <= std::max(q[1], r[1])
          && p[1] >= std::min(q[1], r[1]);
  };

  // Two line segments (p1, q1) and (p2, q2) intersect <=>
  // (p1, q1, p2) and (p1, q1, q2) have different orientations and
  // (p2, q2, p1) and (p2, q2, q1) have different orientations.
  int orientation_p1_q1_p2 = find_orientation(p1, q1, p2);
  int orientation_p1_q1_q2 = find_orientation(p1, q1, q2);
  int orientation_p2_q2_p1 = find_orientation(p2, q2, p1);
  int orientation_p2_q2_q1 = find_orientation(p2, q2, q1);
  if (orientation_p1_q1_p2 != orientation_p1_q1_q2
    && orientation_p2_q2_p1 != orientation_p2_q2_q1) {
    return true;
  }

  // Special cases, two line segments are collinear
  if (orientation_p1_q1_p2 == 0 && is_on_segment(p2, p1, q1)) {
    return true;
  }
  if (orientation_p1_q1_q2 == 0 && is_on_segment(q2, p1, q1)) {
    return true;
  }
  if (orientation_p2_q2_p1 == 0 && is_on_segment(p1, p2, q2)) {
    return true;
  }
  if (orientation_p2_q2_q1 == 0 && is_on_segment(q1, p2, q2)) {
    return true;
  }
  return false;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
