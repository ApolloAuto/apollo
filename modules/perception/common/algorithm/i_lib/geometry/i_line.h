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

#include "modules/perception/common/algorithm/i_lib/algorithm/i_sort.h"
#include "modules/perception/common/algorithm/i_lib/geometry/i_util.h"

namespace apollo {
namespace perception {
namespace algorithm {
// Measure point to line distance in 2D space, point p is in inhomogeneous
// coordinates
template <typename T>
inline T ILineToPointDistance2d(const T *l, const T *p) {
  return IDiv(IAbs(IDot2(l, p) + l[2]), IL2Norm2(l));
}

// Fit a line l: ax+by+c = 0 in 2D space using total least square method.
// The input n 2D points in inhomogeneous coordinates. Array x has size
// 2*n and points are stored as [x0, y0, x1, y1, ...]. x will be destroyed
// after calling this routine.
template <typename T>
inline void ILineFit2dTotalLeastSquare(T *x, T *l, int n) {
  IZero3(l);
  if (n < 2) {
    return;
  }
  T ma[4], eigv[2];
  T mq[4] = {static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0),
             static_cast<T>(0.0)};
  //   //  compute the centroid of input data points
  int i, length = 2 * n;
  T xm = static_cast<T>(0.0);
  T ym = static_cast<T>(0.0);
  for (i = 0; i < length; i += 2) {
    xm += x[i];
    ym += x[i + 1];
  }
  xm = IDiv(xm, n);
  ym = IDiv(ym, n);
  for (i = 0; i < length; i += 2) {
    x[i] -= xm;
    x[i + 1] -= ym;
  }
  IMultAtAnx2(x, ma, n);
  IEigSymmetric2x2Closed(ma, eigv, mq);
  l[0] = mq[1];
  l[1] = mq[3];
  l[2] = -xm * l[0] - ym * l[1];
}

// Fit a line l: ax+by+c = 0 in 2D space with two 2D points in inhomogeneous
// space.
template <typename T>
inline void ILineFit2d(const T *x, const T *xp, T *l) {
  T ma[4] = {x[0], x[1], xp[0], xp[1]};
  ILineFit2dTotalLeastSquare(ma, l, 2);
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
