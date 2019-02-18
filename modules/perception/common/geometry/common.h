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

#include "modules/perception/base/box.h"
#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace common {

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
void CalculateBBoxSizeCenter2DXY(const PointCloudT &cloud,
                                 const Eigen::Vector3f &dir,
                                 Eigen::Vector3f *size, Eigen::Vector3d *center,
                                 float minimum_edge_length = FLT_EPSILON) {
  // NOTE: direction should not be (0, 0, 1)
  Eigen::Matrix3d projection;
  Eigen::Vector3d dird(dir[0], dir[1], 0.0);
  dird.normalize();
  projection << dird[0], dird[1], 0.0, -dird[1], dird[0], 0.0, 0.0, 0.0, 1.0;
  Eigen::Vector3d min_pt(DBL_MAX, DBL_MAX, DBL_MAX);
  Eigen::Vector3d max_pt(-DBL_MAX, -DBL_MAX, -DBL_MAX);
  Eigen::Vector3d loc_pt(0.0, 0.0, 0.0);
  for (int i = 0; i < cloud.size(); i++) {
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

  float minimum_size =
      minimum_edge_length > FLT_EPSILON ? minimum_edge_length : FLT_EPSILON;
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
    const std::vector<base::PointCloud<PointT>> &left_boundary,
    const std::vector<base::PointCloud<PointT>> &right_boundary,
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

}  // namespace common
}  // namespace perception
}  // namespace apollo
