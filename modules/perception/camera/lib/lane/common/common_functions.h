/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <vector>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/point.h"

namespace apollo {
namespace perception {
namespace camera {
typedef std::vector<base::Point3DF> Point3DSet;
typedef std::vector<base::Point2DF> Point2DSet;

static const double lane_eps_value = 1e-6;
static const int max_poly_order = 3;

struct LanePointInfo {
  int type;
  // model output score
  float score;
  // x coordinate
  float x;
  // y coordinate
  float y;
};
// fit polynomial function with QR decomposition (using Eigen 3)
template <typename Dtype>
bool PolyFit(const std::vector<Eigen::Matrix<Dtype, 2, 1>>& pos_vec,
             const int& order,
             Eigen::Matrix<Dtype, max_poly_order + 1, 1>* coeff,
             const bool& is_x_axis = true) {
  if (coeff == nullptr) {
    AERROR << "The coefficient pointer is NULL.";
    return false;
  }

  if (order > max_poly_order) {
    AERROR << "The order of polynomial must be smaller than " << max_poly_order;
    return false;
  }

  int n = static_cast<int>(pos_vec.size());
  if (n <= order) {
    AERROR << "The number of points should be larger than the order. #points = "
           << pos_vec.size();
    return false;
  }

  // create data matrix
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> A(n, order + 1);
  Eigen::Matrix<Dtype, Eigen::Dynamic, 1> y(n);
  Eigen::Matrix<Dtype, Eigen::Dynamic, 1> result;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= order; ++j) {
      A(i, j) = static_cast<Dtype>(
          std::pow(is_x_axis ? pos_vec[i].x() : pos_vec[i].y(), j));
    }
    y(i) = is_x_axis ? pos_vec[i].y() : pos_vec[i].x();
  }

  // solve linear least squares
  result = A.householderQr().solve(y);
  assert(result.size() == order + 1);

  for (int j = 0; j <= max_poly_order; ++j) {
    (*coeff)(j) = (j <= order) ? result(j) : static_cast<Dtype>(0);
  }

  return true;
}

template <typename Dtype>
bool PolyEval(const Dtype& x, int order,
              const Eigen::Matrix<Dtype, max_poly_order + 1, 1>& coeff,
              Dtype* y) {
  int poly_order = order;
  if (order > max_poly_order) {
    AERROR << "the order of polynomial function must be smaller than "
           << max_poly_order;
    return false;
  }

  *y = static_cast<Dtype>(0);
  for (int j = 0; j <= poly_order; ++j) {
    *y += static_cast<Dtype>(coeff(j) * std::pow(x, j));
  }

  return true;
}

// @brief: ransac fitting to estimate the coefficients of linear system
template <typename Dtype>
bool RansacFitting(const std::vector<Eigen::Matrix<Dtype, 2, 1>>& pos_vec,
                   std::vector<Eigen::Matrix<Dtype, 2, 1>>* selected_points,
                   Eigen::Matrix<Dtype, 4, 1>* coeff,
                   const int max_iters = 100,
                   const int N = 5,
                   const Dtype inlier_thres = static_cast<Dtype>(0.1)) {
  if (coeff == nullptr) {
    AERROR << "The coefficient pointer is NULL.";
    return false;
  }

  selected_points->clear();

  int n = static_cast<int>(pos_vec.size());
  int q1 = static_cast<int>(n / 4);
  int q2 = static_cast<int>(n / 2);
  int q3 = static_cast<int>(n * 3 / 4);
  if (n < N) {
    AERROR << "The number of points should be larger than the order. #points = "
           << pos_vec.size();
    return false;
  }

  std::vector<int> index(3, 0);
  int max_inliers = 0;
  Dtype min_residual = FLT_MAX;
  Dtype early_stop_ratio = 0.95f;
  Dtype good_lane_ratio = 0.666f;
  for (int j = 0; j < max_iters; ++j) {
    index[0] = std::rand() % q2;
    index[1] = q2 + std::rand() % q1;
    index[2] = q3 + std::rand() % q1;

    Eigen::Matrix<Dtype, 3, 3> matA;
    matA << pos_vec[index[0]](0) * pos_vec[index[0]](0),
        pos_vec[index[0]](0), 1,
        pos_vec[index[1]](0) * pos_vec[index[1]](0),
        pos_vec[index[1]](0), 1,
        pos_vec[index[2]](0) * pos_vec[index[2]](0),
        pos_vec[index[2]](0), 1;

    Eigen::FullPivLU<Eigen::Matrix<Dtype, 3, 3>> mat(matA);
    mat.setThreshold(1e-5f);
    if (mat.rank() < 3) {
      ADEBUG << "matA: " << matA;
      ADEBUG << "Matrix is not full rank (3). The rank is: " << mat.rank();
      continue;
    }

    // Since Eigen::solver was crashing, simple inverse of 3x3 matrix is used
    // Note that Eigen::inverse of 3x3 and 4x4 is a closed form solution
    Eigen::Matrix<Dtype, 3, 1> matB;
    matB << pos_vec[index[0]](1), pos_vec[index[1]](1), pos_vec[index[2]](1);
    Eigen::Vector3f c =
        static_cast<Eigen::Matrix<Dtype, 3, 1>>(matA.inverse() * matB);
    if (!(matA * c).isApprox(matB)) {
      ADEBUG << "No solution.";
      continue;
    }

    int num_inliers = 0;
    Dtype residual = 0;
    Dtype y = 0;
    for (int i = 0; i < n; ++i) {
      y = pos_vec[i](0) * pos_vec[i](0) * c(0) + pos_vec[i](0) * c(1) + c(2);
      if (std::abs(y - pos_vec[i](1)) <= inlier_thres) ++num_inliers;
      residual += std::abs(y - pos_vec[i](1));
    }

    if (num_inliers > max_inliers ||
        (num_inliers == max_inliers && residual < min_residual)) {
      (*coeff)(3) = 0;
      (*coeff)(2) = c(0);
      (*coeff)(1) = c(1);
      (*coeff)(0) = c(2);
      max_inliers = num_inliers;
      min_residual = residual;
    }

    if (max_inliers > early_stop_ratio * n) break;
  }

  if (static_cast<Dtype>(max_inliers) / n < good_lane_ratio) return false;

  // std::vector<Eigen::Matrix<Dtype, 2, 1>> tmp = *pos_vec;
  // pos_vec.clear();
  for (int i = 0; i < n; ++i) {
    Dtype y = pos_vec[i](0) * pos_vec[i](0) * (*coeff)(2) +
              pos_vec[i](0) * (*coeff)(1) + (*coeff)(0);
    if (std::abs(y - pos_vec[i](1)) <= inlier_thres) {
      selected_points->push_back(pos_vec[i]);
    }
  }
  return true;
}

class DisjointSet {
 public:
  DisjointSet() : disjoint_array_(), subset_num_(0) {}
  explicit DisjointSet(size_t size) : disjoint_array_(), subset_num_(0) {
    disjoint_array_.reserve(size);
  }
  ~DisjointSet() {}

  void Init(size_t size) {
    disjoint_array_.clear();
    disjoint_array_.reserve(size);
    subset_num_ = 0;
  }

  void Reset() {
    disjoint_array_.clear();
    subset_num_ = 0;
  }

  int Add();        // add a new element, which is a subset by itself;
  int Find(int x);  // return the root of x
  void Unite(int x, int y);
  int Size() const { return subset_num_; }
  size_t Num() const { return disjoint_array_.size(); }

 private:
  std::vector<int> disjoint_array_;
  int subset_num_;
};

class ConnectedComponent {
 public:
  ConnectedComponent() : pixel_count_(0) {}

  ConnectedComponent(int x, int y) : pixel_count_(1) {
    base::Point2DI point;
    point.x = x;
    point.y = y;
    pixels_.push_back(point);
    bbox_.xmin = x;
    bbox_.xmax = x;
    bbox_.ymin = y;
    bbox_.ymax = y;
  }

  ~ConnectedComponent() {}

  // CC pixels
  void AddPixel(int x, int y);
  int GetPixelCount() const { return pixel_count_; }
  base::BBox2DI GetBBox() const { return bbox_; }
  std::vector<base::Point2DI> GetPixels() const { return pixels_; }

 private:
  int pixel_count_;
  std::vector<base::Point2DI> pixels_;
  base::BBox2DI bbox_;
};

bool FindCC(const std::vector<unsigned char>& src, int width, int height,
            const base::RectI& roi, std::vector<ConnectedComponent>* cc);

bool ImagePoint2Camera(const base::Point2DF& img_point, float pitch_angle,
                       float camera_ground_height,
                       const Eigen::Matrix3f& intrinsic_params_inverse,
                       Eigen::Vector3d* camera_point);

bool CameraPoint2Image(const Eigen::Vector3d& camera_point,
                       const Eigen::Matrix3f& intrinsic_params,
                       base::Point2DF* img_point);

bool ComparePoint2DY(const base::Point2DF& point1,
                     const base::Point2DF& point2);

template <class T>
void QSwap_(T* a, T* b) {
  T temp = *a;
  *a = *b;
  *b = temp;
}

template <class T>
void QuickSort(int* index, const T* values, int start, int end) {
  if (start >= end - 1) {
    return;
  }

  const T& pivot = values[index[(start + end - 1) / 2]];
  // first, split into two parts: less than the pivot
  // and greater-or-equal
  int lo = start;
  int hi = end;

  for (;;) {
    while (lo < hi && values[index[lo]] < pivot) {
      lo++;
    }
    while (lo < hi && values[index[hi - 1]] >= pivot) {
      hi--;
    }
    if (lo == hi || lo == hi - 1) {
      break;
    }
    QSwap_(&(index[lo]), &(index[hi - 1]));
    lo++;
    hi--;
  }

  int split1 = lo;
  // now split into two parts: equal to the pivot
  // and strictly greater.
  hi = end;
  for (;;) {
    while (lo < hi && values[index[lo]] == pivot) {
      lo++;
    }
    while (lo < hi && values[index[hi - 1]] > pivot) {
      hi--;
    }
    if (lo == hi || lo == hi - 1) {
      break;
    }
    QSwap_(&(index[lo]), &(index[hi - 1]));
    lo++;
    hi--;
  }
  int split2 = lo;
  QuickSort(index, values, start, split1);
  QuickSort(index, values, split2, end);
}

template <class T>
void QuickSort(int* index, const T* values, int nsize) {
  for (int ii = 0; ii < nsize; ii++) {
    index[ii] = ii;
  }
  QuickSort(index, values, 0, nsize);
}
bool FindKSmallValue(const float* distance, int dim, int k, int* index);

bool FindKLargeValue(const float* distance, int dim, int k, int* index);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
