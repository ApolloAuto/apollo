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

#include <utility>

#include "Eigen/Dense"

#include "modules/perception/common/i_lib/core/i_blas.h"

namespace apollo {
namespace perception {
namespace common {

template <typename T>
inline void IEigSymmetric2x2Closed(const T *A, T *EV, T *Q) {
  if (A == nullptr) {
    return;
  }
  Eigen::Matrix<T, 2, 2> a;
  a << A[0], A[1], A[2], A[3];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 2, 2> > es(a);
  if (es.info() != Eigen::Success) {
    return;
  }

  Eigen::Matrix<T, 2, 1> D = es.eigenvalues();
  Eigen::Matrix<T, 2, 2> V = es.eigenvectors();
  if (fabs(D(0, 0)) < fabs(D(1, 0))) {
    std::swap(D(0, 0), D(1, 0));
    std::swap(V(0, 0), V(0, 1));
    std::swap(V(1, 0), V(1, 1));
  }
  EV[0] = D(0, 0);
  EV[1] = D(1, 0);
  Q[0] = V(0, 0);
  Q[1] = V(0, 1);
  Q[2] = V(1, 0);
  Q[3] = V(1, 1);
}

template <typename T>
inline void IEigSymmetric3x3Closed(const T *A, T *EV, T *Q) {
  if (A == nullptr) {
    return;
  }
  Eigen::Matrix<T, 3, 3> a;
  a << A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3> > es(a);
  if (es.info() != Eigen::Success) {
    return;
  }
  Eigen::Matrix<T, 3, 1> D = es.eigenvalues();
  Eigen::Matrix<T, 3, 3> V = es.eigenvectors();
  if (fabs(D(1, 0)) < fabs(D(2, 0))) {
    std::swap(D(1, 0), D(2, 0));
    for (int i = 0; i < 3; ++i) {
      std::swap(V(i, 1), V(i, 2));
    }
  }
  if (fabs(D(0, 0)) < fabs(D(1, 0))) {
    std::swap(D(0, 0), D(1, 0));
    for (int i = 0; i < 3; ++i) {
      std::swap(V(i, 0), V(i, 1));
    }
  }
  if (fabs(D(1, 0)) < fabs(D(2, 0))) {
    std::swap(D(1, 0), D(2, 0));
    for (int i = 0; i < 3; ++i) {
      std::swap(V(i, 1), V(i, 2));
    }
  }

  EV[0] = D(0, 0);
  EV[1] = D(1, 0);
  EV[2] = D(2, 0);
  Q[0] = V(0, 0);
  Q[1] = V(0, 1);
  Q[2] = V(0, 2);
  Q[3] = V(1, 0);
  Q[4] = V(1, 1);
  Q[5] = V(1, 2);
  Q[6] = V(2, 0);
  Q[7] = V(2, 1);
  Q[8] = V(2, 2);
}

// Compute x=[R|t]*X, assuming R is 3x3 rotation matrix and t is a 3-vector.
template <typename T>
inline void IProjectThroughExtrinsic(const T *R, const T *t, const T *X, T *x) {
  IMultAx3x3(R, X, x);
  IAdd3(t, x);
}

// Compute x=K*X, assuming K is 3x3 upper triangular with K[8] = 1.0, do not
// consider radial distortion.
template <typename T>
inline void IProjectThroughIntrinsic(const T *K, const T *X, T *x) {
  x[0] = K[0] * X[0] + K[1] * X[1] + K[2] * X[2];
  x[1] = K[4] * X[1] + K[5] * X[2];
  x[2] = X[2];
}

// Compute x=K[R|t]*X, assuming K is 3x3 upper triangular with K[8] = 1.0,
// assuming R is 3x3 rotation matrix and t is a 3-vector, do not consider
// radial distortion.
template <typename T>
inline void IProjectThroughKRt(const T *K, const T *R, const T *t, const T *X,
                               T *x) {
  T Rtx[3];
  IProjectThroughExtrinsic(R, t, X, Rtx);
  IProjectThroughIntrinsic(K, Rtx, x);
}

// If K is
// [fx  s  cx]
// [0  fy  cy]
// [0   0   1]
// then K^-1 is
// [1/fx  -s/(fy*fx)     s*cy/(fy*fx)-cx/fx]
// [0       1/fy                -cy/fy     ]
// [0        0                     1       ]
template <typename T>
inline void IUnprojectThroughIntrinsic(const T *K, const T *x, T *X) {
  T fx_rec = IRec(K[0]);
  T fy_rec = IRec(K[4]);
  T fxfy_rec = IRec(K[0] * K[4]);
  X[0] = fx_rec * x[0] - (K[1] * fxfy_rec) * x[1] +
         (K[1] * K[5] * fxfy_rec - K[2] * fx_rec) * x[2];
  X[1] = fy_rec * x[1] - (K[5] * fy_rec) * x[2];
  X[2] = x[2];
}

// Check if a point X is in front of the camera, first argument is the center
// of projection, second is the principal ray (axis) vector
template <typename T>
inline bool IPointInFrontOfCamera(const T *X, const T *cop, const T *prv) {
  T Xmcop[3], v_norm;
  ISub3(X, cop, Xmcop);
  v_norm = IL2Norm3(Xmcop);
  if (v_norm == static_cast<T>(0.0)) {
    return false;  // X is the cop
  }
  IScale3(Xmcop, IRec(v_norm));
  return IDot3(Xmcop, prv) > static_cast<T>(0.0);
}

// Back project a 2D point x to 3D X given it's depth Z. Assume the camera is
// canonical K[I|0], K with 0 skew.
template <typename T>
inline void IBackprojectCanonical(const T *x, const T *K, T depth, T *X) {
  X[0] = (x[0] - K[2]) * depth * IRec(K[0]);
  X[1] = (x[1] - K[5]) * depth * IRec(K[4]);
  X[2] = depth;
}

// Back project a 2D point x to 3D X given it's depth Z. Assume the camera is
//  * K[R|t], K with 0 skew.
template <typename T>
inline void IBackprojectThroughKRt(const T *x, const T *K, const T *R,
                                   const T *t, T depth, T *X) {
  // K^-1*[u,v,1]':
  T u = (x[0] - K[2]) * IRec(K[0]);
  T v = (x[1] - K[5]) * IRec(K[4]);
  // R^-1*K^-1*[u,v,1]':
  T ru = R[0] * u + R[3] * v + R[6];
  T rv = R[1] * u + R[4] * v + R[7];
  T rw = R[2] * u + R[5] * v + R[8];
  // R^-1*t:
  T tu = R[0] * t[0] + R[3] * t[1] + R[6] * t[2];
  T tv = R[1] * t[0] + R[4] * t[1] + R[7] * t[2];
  T tw = R[2] * t[0] + R[5] * t[1] + R[8] * t[2];
  T w = IDiv(depth + tw, rw);  // scale factor
  X[0] = ru * w - tu;
  X[1] = rv * w - tv;
  X[2] = depth;
}

// Back project a 2D point x to 3D X given a plane equation pi in 3D. Assume
// the camera is canonical K[I|0] (K with 0 skew) and the point X lies on plane
// pi. The routine returns true if there is a valid intersection, otherwise
// (e.g.,  ray is nearly parallel to plane) returns false.
template <typename T>
inline bool IBackprojectPlaneIntersectionCanonical(const T *x, const T *K,
                                                   const T *pi, T *X) {
  IZero3(X);
  T umcx = IDiv(x[0] - K[2], K[0]);
  T vmcy = IDiv(x[1] - K[5], K[4]);
  T sf = pi[0] * umcx + pi[1] * vmcy + pi[2];
  if (sf == static_cast<T>(0.0)) {
    return false;
  }
  T Z = -pi[3] / sf;
  X[0] = Z * umcx;
  X[1] = Z * vmcy;
  X[2] = Z;
  return Z > static_cast<T>(0.0);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
