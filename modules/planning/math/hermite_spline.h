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
 * @file hermite_spline.h
 **/

#ifndef MODULES_PLANNING_MATH_HERMITE_SPLINE_H_
#define MODULES_PLANNING_MATH_HERMITE_SPLINE_H_

#include <utility>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

template<typename T, std::uint32_t N>
class HermiteSpline {
 public:
  HermiteSpline(std::array<T, (N + 1) / 2> x0, std::array<T, (N + 1) / 2> x1,
                const double z0 = 0.0, const double z1 = 1.0);

  virtual ~HermiteSpline() = default;

  virtual T evaluate(const std::uint32_t order, const double z) const;

  // virtual std::array<T, N> evaluate(const double z) const;
 private:
  std::array<T, (N + 1) / 2> _x0;

  std::array<T, (N + 1) / 2> _x1;

  double _z0 = 0.0;

  double delta_z_ = 0.0;
};

template<typename T, std::uint32_t N>
inline HermiteSpline<T, N>::HermiteSpline(std::array<T, (N + 1) / 2> x0,
                                          std::array<T, (N + 1) / 2> x1,
                                          const double z0, const double z1)
    : _x0(std::move(x0)), _x1(std::move(x1)), _z0(z0), delta_z_(z1 - z0) {
  CHECK(N == 3 || N == 5)
  << "Error: currently we only support cubic and quintic hermite splines!";
}

template<typename T, std::uint32_t N>
inline T HermiteSpline<T, N>::evaluate(const std::uint32_t order,
                                       const double z) const {
  CHECK_LE(_z0, z);
  CHECK_LE(z, _z0 + delta_z_);

  // if N == 3, cubic hermite spline, N == 5, qunitic hermite spline
  if (N == 3) {
    double p0 = _x0[0];
    double v0 = _x0[1];
    double p1 = _x1[0];
    double v1 = _x1[1];
    switch (order) {
      case 0: {
        double t = (z - _z0) / delta_z_;
        double t2 = t * t;
        double t3 = t2 * t;

        return (2.0 * t3 - 3.0 * t2 + 1.0) * p0 + (t3 - 2 * t2 + t) * v0 +
            (-2.0 * t3 + 3.0 * t2) * p1 + (t3 - t2) * v1;
      }
      case 1: {
        double t = (z - _z0) / delta_z_;
        double t2 = t * t;

        return (6.0 * t2 - 6.0 * t) * p0 + (3.0 * t2 - 4 * t + 1.0) * v0 +
            (-6.0 * t2 + 6.0 * t) * p1 + (3.0 * t2 - 2.0 * t) * v1;
      }
      case 2: {
        double t = (z - _z0) / delta_z_;
        return (12.0 * t - 6.0) * p0 + (6.0 * t - 4.0) * v0 +
            (-12.0 * t + 6.0) * p1 + (6.0 * t - 2.0) * v1;
      }
      case 3: {
        return 12.0 * p0 + 6.0 * v0 - 12.0 * p1 + 6.0 * v1;
      }
      default: { break; }
    }
  } else {
    CHECK_EQ(5, N);
    double p0 = _x0[0];
    double v0 = _x0[1];
    double a0 = _x0[2];
    double p1 = _x1[0];
    double v1 = _x1[1];
    double a1 = _x1[2];

    switch (order) {
      case 0: {
        double t = (z - _z0) / delta_z_;
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t2 * t2;
        double t5 = t2 * t3;
        double det0 = t3 - t4;
        double det1 = t4 - t5;
        double h0 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5;
        double h1 = t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5;
        double h2 = 0.5 * (t2 - t5) - 1.5 * det0;
        double h3 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
        double h4 = -4.0 * det0 + 3.0 * det1;
        double h5 = 0.5 * (det0 - det1);

        return h0 * p0 + h1 * v0 + h2 * a0 + h3 * p1 + h4 * v1 + h5 * a1;
      }
      case 1: {
        double t = (z - _z0) / delta_z_;
        double t2 = t * t;
        double t3 = t * t2;
        double t4 = t2 * t2;
        double det0 = t2 - t3;
        double det1 = t3 - t4;
        double dh0 = -30.0 * det0 + 30.0 * det1;
        double dh1 = 1 - 18.0 * t2 + 32.0 * t3 - 15.0 * t4;
        double dh2 = t - 4.5 * t2 + 6.0 * t3 - 2.5 * t4;
        double dh3 = 30.0 * det0 - 30.0 * det1;
        double dh4 = -12.0 * t2 + 28.0 * t3 - 15.0 * t4;
        double dh5 = 1.5 * det0 - 2.5 * det1;

        return dh0 * p0 + dh1 * v0 + dh2 * a0 + dh3 * p1 + dh4 * v1 + dh5 * a1;
      }
      case 2: {
        double t = (z - _z0) / delta_z_;
        double t2 = t * t;
        double t3 = t * t2;
        double det0 = t - t2;
        double det1 = t2 - t3;
        double ddh0 = -60.0 * det0 + 120.0 * det1;
        double ddh1 = -36.0 * det0 + 60.0 * det1;
        double ddh2 = 1.0 - 9.0 * t + 18.0 * t2 - 10.0 * t3;
        double ddh3 = 60.0 * det0 - 120.0 * det1;
        double ddh4 = -24.0 * det0 + 60.0 * det1;
        double ddh5 = 3.0 * t - 12.0 * t2 + 10.0 * t3;

        return ddh0 * p0 + ddh1 * v0 + ddh2 * a0 + ddh3 * p1 + ddh4 * v1 +
            ddh5 * a1;
      }
      case 3: {
        double t = (z - _z0) / delta_z_;
        double t2 = t * t;
        double det = t - t2;
        double dddh0 = -60.0 + 360.0 * det;
        double dddh1 = -36.0 + 192.0 * t - 180.0 * t2;
        double dddh2 = -9.0 + 36.0 * t - 30.0 * t2;
        double dddh3 = 60.0 - 360.0 * det;
        double dddh4 = -24.0 + 168.0 * t - 180.0 * t2;
        double dddh5 = 3.0 - 24.0 * t + 30.0 * t2;

        return dddh0 * p0 + dddh1 * v0 + dddh2 * a0 + dddh3 * p1 + dddh4 * v1 +
            dddh5 * a1;
      }
        // TODO(fanhaoyang): the derive higher order derivative for
        // quintic hermite spline
      default: { break; }
    }
  }
  AFATAL << "Error: unsupported order of spline or derivative!";
  T t;
  return t;
}

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_MATH_HERMITE_SPLINE_H_ */
