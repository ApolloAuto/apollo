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

#include "modules/perception/common/i_lib/algorithm/i_sort.h"
#include "modules/perception/common/i_lib/core/i_alloc.h"
#include "modules/perception/common/i_lib/da/i_ransac.h"
#include "modules/perception/common/i_lib/geometry/i_line.h"
#include "modules/perception/common/i_lib/geometry/i_util.h"

namespace apollo {
namespace perception {
namespace common {
// General plane equation pi: ax+by+cz+d = 0

// Another 3D Plane Data structure:
template <typename T>
struct PlanePara {
  PlanePara(const PlanePara<T> &pi) {
    ICopy4(pi.p, p);
    ICopy3(pi.t, t);
    ICopy2(pi.data_stat, data_stat);
  }
  PlanePara &operator=(const PlanePara<T> &pi) {
    ICopy4(pi.p, this->p);
    ICopy3(pi.t, this->t);
    ICopy2(pi.data_stat, this->data_stat);
    return (*this);
  }
  // first 3 elements: normal with unit norm, last elemet: d
  T p[4];
  T t[3];  // centroid
           // mean distance and standard deviation of
           // the point to plane distance distribution
  T data_stat[2];
  void Clear() {
    IZero4(p);
    IZero3(t);
    IZero2(data_stat);
  }
  void Assign(const T *pi) {
    ICopy4(pi, p);
    ICopy3(pi + 4, t);
    ICopy2(pi + 7, data_stat);
  }
  bool IsValid() { return (ISqrt(IQquaresum3(p)) > static_cast<T>(0)); }
  PlanePara() { Clear(); }
};

// Compute the projection (q) of a 3D point (p) on a plane (pi) in 3D space
template <typename T>
inline void IPointOnPlaneProjection(const T *pi, const T *p, T *q) {
  T npi[4];
  ICopy4(pi, npi);
  T sf = IRec(IL2Norm3(npi));
  IScale4(npi, sf);  // plane with unit norm
  sf = -(IDot3(npi, p) + npi[3]);
  q[0] = p[0] + sf * npi[0];
  q[1] = p[1] + sf * npi[1];
  q[2] = p[2] + sf * npi[2];
}

// Measure point to plane distance in 3D space, point p is in inhomogeneous
// coordinates
template <typename T>
inline T IPlaneToPointDistance(const T *pi, const T *p) {
  return IDiv(IAbs(IDot3(pi, p) + pi[3]), IL2Norm3(pi));
}

// Measure point to plane distance in 3D space, point p is in inhomogeneous
// coordinates
template <typename T>
inline T IPlaneToPointDistance(const T *pi, const T *p, T l2_norm3_pi) {
  return IDiv(IAbs(IDot3(pi, p) + pi[3]), l2_norm3_pi);
}

// Measure point to plane distance in 3D space, point p is in inhomogeneous
// coordinates
template <typename T>
inline T IPlaneToPointDistanceWUnitNorm(const T *pi, const T *p) {
  return IAbs(IDot3(pi, p) + pi[3]);
}

// Measure point to plane distance (signed) in 3D space, point p is in
// inhomogeneous coordinates
// distance is positive if p is on the same side of the plane pi as the normal
// vector and negative if it is on the opposite side
template <typename T>
inline T IPlaneToPointSignedDistanceWUnitNorm(const T *pi, const T *p) {
  return IDot3(pi, p) + pi[3];
}

// Measure point to plane distance in 3D space, point p is in inhomogeneous
// coordinates
template <typename T>
inline T IPlaneToPointDistanceWNormalizedPlaneNorm(const T *pi, const T *p) {
  return IAbs(IDot3(pi, p) + pi[3]);
}

// Measure the normal angle in degree between two planes
template <typename T>
inline T IPlaneToPlaneNormalDeltaDegreeZUp(const T *pi_p, const T *pi_q) {
  T normal_p[3], normal_q[3];
  ICopy3(pi_p, normal_p);
  ICopy3(pi_q, normal_q);

  // Z is up
  if (normal_p[2] < static_cast<T>(0.0)) {
    INeg3(normal_p);
  }
  if (normal_q[2] < static_cast<T>(0.0)) {
    INeg3(normal_q);
  }

  IScale3(normal_p, IRec(ISqrt(IQquaresum3(normal_p))));  // normalize
  IScale3(normal_q, IRec(ISqrt(IQquaresum3(normal_q))));  // normalize
  return IRadiansToDegree(IAcos(IDot3(normal_p, normal_q)));
}

// Fit a plane pi (ax+by+cz+d = 0): in 3D space using
// total least square method (Orthogonal Distance Regression). The input n 3D
// points are in  inhomogeneous coordinates. Array X has Size n * 3 and points
// are stored as[x0,y0, z0, x1, y1, z1, ...].
// pi is stored as 4 - vector[a, b, c, d]. x will be destroyed after calling
// this routine.
template <typename T>
inline void IPlaneFitTotalLeastSquare(T *X, T *pi, int n) {
  IZero4(pi);
  if (n < 3) {
    return;
  }
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  // compute the centroid of input Data points
  T xm = static_cast<T>(0.0);
  T ym = static_cast<T>(0.0);
  T zm = static_cast<T>(0.0);
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= static_cast<T>(n);
  ym /= static_cast<T>(n);
  zm /= static_cast<T>(n);
  for (i = 0, j = 0; i < n; i++) {
    X[j++] -= xm;
    X[j++] -= ym;
    X[j++] -= zm;
  }
  IMultAtAnx3(X, mat_a, n);
  IEigSymmetric3x3Closed(mat_a, eigv, mat_q);
  // pi's normal vector is the eigen vector of
  // mat_a corresponding to its smallest eigen value
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  // the optimal plane should pass [xm, ym, zm]:
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);
}

// Fit a plane pi (ax+by+cz+d = 0): in 3D space using
// total least square method (Orthogonal Distance Regression). The input n 3D
// points are in inhomogeneous coordinates. Array X has Size 3 * 3 and points
// are stored as[x0, y0, z0, x1, y1, z1, ...].
// pi is stored as 4 - vector[a, b, c, d]. x will be destroyed after calling
// this routine.
template <typename T>
inline void IPlaneFitTotalLeastSquare3(T *X, T *pi) {
  T mat_a[9];
  T eigv[3];
  T mat_q[9];
  // compute the centroid of input Data points
  IZero4(pi);
  T xm = (X[0] + X[3] + X[6]) / 3;
  T ym = (X[1] + X[4] + X[7]) / 3;
  T zm = (X[2] + X[5] + X[8]) / 3;
  X[0] -= xm;
  X[1] -= ym;
  X[2] -= zm;
  X[3] -= xm;
  X[4] -= ym;
  X[5] -= zm;
  X[6] -= xm;
  X[7] -= ym;
  X[8] -= zm;
  IMultAtA3x3(X, mat_a);
  IEigSymmetric3x3Closed(mat_a, eigv, mat_q);
  // pi's normal vector is the eigen vector of
  // mat_a corresponding to its smallest eigen value
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  // the optimal plane should pass [xm, ym, zm]:
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);
}

// Fit a plane pi: {[a,b,c,d], [x,y,z]} in 3D space using
// total least square method (Orthogonal Distance Regression). The input n 3D
// points are in inhomogeneous coordinates. Array X has Size n * 3 and points
// are stored as[x0, y0, z0, x1, y1, z1, ...].
// The routine needs n * 3 positions of scratch space in Xp.
template <typename T>
inline void IPlaneFitTotalLeastSquare(const T *X, T *pi, int n, T *Xp,
                                      T *centroid, T *err_stat) {
  IZero4(pi);
  if (centroid != nullptr) {
    IZero3(centroid);
  }
  if (err_stat != nullptr) {
    IZero2(err_stat);
  }
  if (n < 3) {
    return;
  }

  int i, j, n3 = n * 3;
  T mat_a[9], eigv[3], mat_q[9], t[3];
  // compute the centroid of input Data points
  T xm = static_cast<T>(0.0);
  T ym = static_cast<T>(0.0);
  T zm = static_cast<T>(0.0);
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  t[0] = xm / n;
  t[1] = ym / n;
  t[2] = zm / n;
  for (i = 0; i < n3; i += 3) {
    ISub3(X + i, t, Xp + i);
  }
  IMultAtANx3(Xp, mat_a, n);
  IEigSymmetric3x3Closed(mat_a, eigv, mat_q);
  // pi's normal vector is the eigen vector of
  // mat_a corresponding to its smallest eigen value
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  IUnitize3(pi);
  // the optimal plane should pass [xm, ym, zm]:
  pi[3] = -IDot3(pi, t);

  if (centroid != nullptr) {
    ICopy3(t, centroid);
  }

  // Data stat:
  if (err_stat != nullptr) {
    const T *cptr_data = X;
    for (i = 0; i < n; ++i) {
      Xp[i] = IPlaneToPointDistance(pi, cptr_data);
      cptr_data += 3;
    }
    err_stat[0] = IMean(Xp, n);              // mean
    err_stat[1] = ISdv(Xp, err_stat[0], n);  // sdv
  }
}

// Fit a plane pi (ax+by+cz+d = 0): in 3D space using
// total least square method (Orthogonal Distance Regression). The input n 3D
// points are in inhomogeneous coordinates. Array X has Size n * 3 and points
// are stored as[x0, y0, z0, x1, y1, z1, ...].
// pi is stored as 4 - vector[a, b, c, d]. x and indices will be destroyed after
// calling this routine.
template <typename T>
inline void IPlaneFitTotalLeastSquare(T *X, int *indices, T *pi, int m, int n) {
  IZero4(pi);
  if (n < 3 || n > m) {
    return;
  }
  IIndexedShuffle(X, indices, m, 3, n);
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  // compute the centroid of input Data points
  T xm = static_cast<T>(0.0);
  T ym = static_cast<T>(0.0);
  T zm = static_cast<T>(0.0);
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    X[j++] -= xm;
    X[j++] -= ym;
    X[j++] -= zm;
  }
  IMultAtANx3(X, mat_a, n);
  IEigSymmetric3x3Closed(mat_a, eigv, mat_q);
  // pi's normal vector is the eigen vector of
  // mat_a corresponding to its smallest eigen value
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  // the optimal plane should pass [xm, ym, zm]:
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);
}

// Fit a plane pi (ax+by+cz+d = 0): in 3D space using
// total least square method (Orthogonal Distance Regression). The input n 3D
// points are in inhomogeneous coordinates. Array X has Size n * 3 and points
// are stored as[x0, y0, z0, x1, y1, z1, ...].
// pi is stored as 4 - vector[a, b, c, d]. x and indices will be destroyed after
// calling this routine.
template <typename T>
inline void IPlaneFitTotalLeastSquare(T *X, int *indices, T *pi, T *centroid,
                                      int m, int n) {
  IZero4(pi);
  if (n < 3 || n > m) {
    return;
  }
  IIndexedShuffle(X, indices, m, 3, n);
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  // compute the centroid of input Data points
  T xm = static_cast<T>(0.0);
  T ym = static_cast<T>(0.0);
  T zm = static_cast<T>(0.0);
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    X[j++] -= xm;
    X[j++] -= ym;
    X[j++] -= zm;
  }
  IMultAtANx3(X, mat_a, n);
  IEigSymmetric3x3Closed(mat_a, eigv, mat_q);
  // pi's normal vector is the eigen vector of
  // mat_a corresponding to its smallest eigen value
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  // the optimal plane should pass [xm, ym, zm]:
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);

  if (centroid != nullptr) {
    centroid[0] = xm;
    centroid[1] = ym;
    centroid[2] = zm;
  }
}

// Fit a plane i: {[a,b,c,d], [x,y,z]}  in 3D space using
// total least square method (Orthogonal Distance Regression). The input n 3D
// points are in inhomogeneous coordinates. Array X has Size n * 3 and points
// are stored as[x0, y0, z0, x1, y1, z1, ...].
template <typename T>
inline void IPlaneFitTotalLeastSquareAdv(T *X, int *indices, T *para, int m,
                                         int n) {
  IZero9(para);
  if (n < 3 || n > m) {
    return;
  }
  int i, j;
  T *xp = IAlloc<T>(n * 3);
  for (i = 0; i < n; i++) {
    j = indices[i];
    ICopy3(X + j * 3, xp + i * 3);
  }
  T mat_a[9], eigv[3], mat_q[9];
  // compute the centroid of input Data points
  T xm = static_cast<T>(0.0);
  T ym = static_cast<T>(0.0);
  T zm = static_cast<T>(0.0);
  for (i = 0, j = 0; i < n; i++) {
    xm += xp[j++];
    ym += xp[j++];
    zm += xp[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    xp[j++] -= xm;
    xp[j++] -= ym;
    xp[j++] -= zm;
  }
  IMultAtANx3(xp, mat_a, n);
  IEigSymmetric3x3Closed(mat_a, eigv, mat_q);
  // pi's normal vector is the eigen vector of
  // mat_a corresponding to its smallest eigen value
  para[0] = mat_q[2];
  para[1] = mat_q[5];
  para[2] = mat_q[8];
  IUnitize3(para);
  // the optimal plane should pass [xm, ym, zm]:
  para[4] = xm;
  para[5] = ym;
  para[6] = zm;
  para[3] = -IDot3(para, para + 4);
  // Data stat:

  for (i = 0; i < n; ++i) {
    j = indices[i];
    xp[i] = IPlaneToPointDistance(para, X + j * 3);
  }
  para[7] = IMean(xp, n);
  para[8] = ISdv(xp, para[7], n);
  IFree<T>(&xp);
}

// Fit a plane pi (ax+by+cz+d = 0) with three 3D points in inhomogeneous
// space - minimal solution
template <typename T>
inline void IPlaneFit(const T *X1, const T *X2, const T *X3, T *pi) {
  T mat_a[9] = {X1[0], X1[1], X1[2], X2[0], X2[1], X2[2], X3[0], X3[1], X3[2]};
  IPlaneFitTotalLeastSquare(mat_a, pi, 3);
}

// Fit a plane pi (ax+by+cz+d = 0) with three 3D points in inhomogeneous
// space - minimal solution
template <typename T>
inline void IPlaneFit(const T *Xs, T *pi) {
  T mat_a[9];
  ICopy9(Xs, mat_a);
  IPlaneFitTotalLeastSquare(mat_a, pi, 3);
}

// Fit a plane pi (ax+by+cz+d = 0) with three 3D points in inhomogeneous
// space - minimal solution,
// note that the input Xs will be destroyed
template <typename T>
inline void IPlaneFitDestroyed(T *Xs, T *pi) {
  IPlaneFitTotalLeastSquare3(Xs, pi);
}

// Fit a plane pi: {[a,b,c,d], [x,y,z]} with three 3D points in inhomogeneous
// space - minimal solution
template <typename T>
inline void IPlaneFitAdv(const T *Xs, T *para) {
  T mat_a[9], mat_ap[9];
  PlanePara<T> pi;
  ICopy9(Xs, mat_a);
  IPlaneFitTotalLeastSquare(mat_a, pi, mat_ap, 3);
  ICopy4(pi.p, para);
  ICopy3(pi.t, para + 4);
  ICopy2(pi.data_stat, para + 7);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
