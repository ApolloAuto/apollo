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
 * @file quintic_polynomial_curve1d.cc
 **/

#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {

QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 3>& end,
    const double param)
    : QuinticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                               end[2], param) {}

QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double ddx1, const double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_[0] = x1;
  end_condition_[1] = dx1;
  end_condition_[2] = ddx1;
  param_ = param;
}

QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
    const QuinticPolynomialCurve1d& other) {
  param_ = other.param_;
  coef_ = other.coef_;
}

double QuinticPolynomialCurve1d::Evaluate(const uint32_t order,
                                          const double p) const {
  switch (order) {
    case 0: {
      return ((((coef_[5] * p + coef_[4]) * p + coef_[3]) * p + coef_[2]) * p +
              coef_[1]) *
                 p +
             coef_[0];
    }
    case 1: {
      return (((5.0 * coef_[5] * p + 4.0 * coef_[4]) * p + 3.0 * coef_[3]) * p +
              2.0 * coef_[2]) *
                 p +
             coef_[1];
    }
    case 2: {
      return (((20.0 * coef_[5] * p + 12.0 * coef_[4]) * p) + 6.0 * coef_[3]) *
                 p +
             2.0 * coef_[2];
    }
    case 3: {
      return (60.0 * coef_[5] * p + 24.0 * coef_[4]) * p + 6.0 * coef_[3];
    }
    case 4: {
      return 120.0 * coef_[5] * p + 24.0 * coef_[4];
    }
    case 5: {
      return 120.0 * coef_[5];
    }
    default:
      return 0.0;
  }
}

void QuinticPolynomialCurve1d::SetParam(const double x0, const double dx0,
                                        const double ddx0, const double x1,
                                        const double dx1, const double ddx1,
                                        const double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
  param_ = param;
}

void QuinticPolynomialCurve1d::IntegratedFromQuarticCurve(
    const PolynomialCurve1d& other, const double init_value) {
  CHECK_EQ(other.Order(), 4);
  param_ = other.ParamLength();
  coef_[0] = init_value;
  for (size_t i = 0; i < 5; ++i) {
    coef_[i + 1] = other.Coef(i) / (static_cast<double>(i) + 1);
  }
}

void QuinticPolynomialCurve1d::ComputeCoefficients(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double ddx1, const double p) {
  CHECK_GT(p, 0.0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = ddx0 / 2.0;

  const double p2 = p * p;
  const double p3 = p * p2;

  // the direct analytical method is at least 6 times faster than using matrix
  // inversion.
  const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
  const double c1 = (dx1 - ddx0 * p - dx0) / p2;
  const double c2 = (ddx1 - ddx0) / p;

  coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
  coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
  coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
}

std::string QuinticPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

double QuinticPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(6, order);
  return coef_[order];
}
}  // namespace planning
}  // namespace apollo
