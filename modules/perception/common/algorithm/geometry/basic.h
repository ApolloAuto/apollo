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

#include <limits>

#include "Eigen/Core"
#include "modules/perception/common/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace algorithm {

// @brief cross production on two vectors
// one is from pt1 to pt2, another is from pt1 to pt3
// the type of points could be double or float
// old name: cross_prod
template <typename Type>
inline Type CrossProduct(const Eigen::Matrix<Type, 2, 1> &point1,
                         const Eigen::Matrix<Type, 2, 1> &point2,
                         const Eigen::Matrix<Type, 2, 1> &point3) {
  return (point2.x() - point1.x()) * (point3.y() - point1.y()) -
         (point3.x() - point1.x()) * (point2.y() - point1.y());
}

// @brief cross production on two vectors
// one is from pt1 to pt2, another is from pt1 to pt3
// the type of points could be double or float
// old name: cross_prod
template <typename PointT>
inline typename PointT::Type CrossProduct(const PointT &point1,
                                          const PointT &point2,
                                          const PointT &point3) {
  return (point2.x - point1.x) * (point3.y - point1.y) -
         (point3.x - point1.x) * (point2.y - point1.y);
}

// @brief calculate the Eucliden distance between two points
// old name: euclidean_dist
template <typename PointT>
inline typename PointT::Type CalculateEuclidenDist(const PointT &pt1,
                                                   const PointT &pt2) {
  typename PointT::Type dist = (pt1.x - pt2.x) * (pt1.x - pt2.x);
  dist += (pt1.y - pt2.y) * (pt1.y - pt2.y);
  dist += (pt1.z - pt2.z) * (pt1.z - pt2.z);
  return static_cast<typename PointT::Type>(sqrt(dist));
}

// @brief calculate the Euclidean distance between two points in X-Y plane
// old name: euclidean_dist_2d_xy
template <typename PointT>
inline typename PointT::Type CalculateEuclidenDist2DXY(const PointT &pt1,
                                                       const PointT &pt2) {
  typename PointT::Type dist = (pt1.x - pt2.x) * (pt1.x - pt2.x);
  dist += (pt1.y - pt2.y) * (pt1.y - pt2.y);
  return static_cast<typename PointT::Type>(sqrt(dist));
}

// @brief calculate cos value of the rotation angle
// between two vectors in X-Y plane
// old name: vector_cos_theta_2d_xy
template <typename T>
T CalculateCosTheta2DXY(const Eigen::Matrix<T, 3, 1> &v1,
                        const Eigen::Matrix<T, 3, 1> &v2) {
  T v1_len = static_cast<T>(sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum()));
  T v2_len = static_cast<T>(sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum()));
  if (v1_len < std::numeric_limits<T>::epsilon() ||
      v2_len < std::numeric_limits<T>::epsilon()) {
    return 0.0;
  }
  T cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  return cos_theta;
}

// @brief calculate the rotation angle between two vectors in X-Y plane
// old name: vector_theta_2d_xy
template <typename T>
T CalculateTheta2DXY(const Eigen::Matrix<T, 3, 1> &v1,
                     const Eigen::Matrix<T, 3, 1> &v2) {
  T v1_len = static_cast<T>(sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum()));
  T v2_len = static_cast<T>(sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum()));
  if (v1_len < std::numeric_limits<T>::epsilon() ||
      v2_len < std::numeric_limits<T>::epsilon()) {
    return 0.0;
  }
  const T cos_theta =
      (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  const T sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);
  T theta = std::acos(cos_theta);
  if (sin_theta < 0.0) {
    theta = -theta;
  }
  if (std::isnan(theta) || std::isinf(theta)) {
    theta = 0.0;
  }
  return theta;
}

// @brief calculate the rotation matrix
// transform from v1 axis coordinate to v2 axis coordinate
// old name: vector_rot_mat_2d_xy
template <typename T>
Eigen::Matrix<T, 3, 3> CalculateRotationMat2DXY(
    const Eigen::Matrix<T, 3, 1> &v1, const Eigen::Matrix<T, 3, 1> &v2) {
  T v1_len = static_cast<T>(sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum()));
  T v2_len = static_cast<T>(sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum()));
  if (v1_len < std::numeric_limits<T>::epsilon() ||
      v2_len < std::numeric_limits<T>::epsilon()) {
    return Eigen::Matrix<T, 3, 3>::Zero(3, 3);
  }

  const T cos_theta =
      (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  const T sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);

  Eigen::Matrix<T, 3, 3> rot_mat;
  rot_mat << cos_theta, sin_theta, 0, -sin_theta, cos_theta, 0, 0, 0, 1;
  return rot_mat;
}

// @brief calculate the project vector from one vector to another
// old name: compute_2d_xy_project_vector
template <typename T>
Eigen::Matrix<T, 3, 1> Calculate2DXYProjectVector(
    const Eigen::Matrix<T, 3, 1> &projected_vector,
    const Eigen::Matrix<T, 3, 1> &project_vector) {
  if (projected_vector.head(2).norm() < std::numeric_limits<T>::epsilon() ||
      project_vector.head(2).norm() < std::numeric_limits<T>::epsilon()) {
    return Eigen::Matrix<T, 3, 1>::Zero(3, 1);
  }
  Eigen::Matrix<T, 3, 1> project_dir = project_vector;
  project_dir(2) = 0.0;
  project_dir.normalize();

  const T projected_vector_project_dir_inner_product =
      projected_vector(0) * project_dir(0) +
      projected_vector(1) * project_dir(1);
  const T projected_vector_project_dir_angle_cos =
      projected_vector_project_dir_inner_product /
      (projected_vector.head(2).norm() * project_dir.head(2).norm());
  const T projected_vector_norm_on_project_dir =
      projected_vector.head(2).norm() * projected_vector_project_dir_angle_cos;
  return project_dir * projected_vector_norm_on_project_dir;
}

// @brief convert point xyz in Cartesian coordinate to polar coordinate
// old name: xyz_to_polar_coordinate
template <typename PointT>
void ConvertCartesiantoPolarCoordinate(const PointT &xyz,
                                       typename PointT::Type *h_angle_in_degree,
                                       typename PointT::Type *v_angle_in_degree,
                                       typename PointT::Type *dist) {
  using T = typename PointT::Type;
  const T radian_to_degree = 180.0 / M_PI;
  const T x = xyz.x;
  const T y = xyz.y;
  const T z = xyz.z;

  (*dist) = static_cast<T>(sqrt(x * x + y * y + z * z));
  T dist_xy = static_cast<T>(sqrt(x * x + y * y));

  (*h_angle_in_degree) = std::acos(x / dist_xy) * radian_to_degree;
  if (y < 0.0) {
    (*h_angle_in_degree) = static_cast<T>(360.0) - (*h_angle_in_degree);
  }

  (*v_angle_in_degree) = std::acos(dist_xy / (*dist)) * radian_to_degree;
  if (z < 0.0) {
    (*v_angle_in_degree) = -(*v_angle_in_degree);
  }
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
