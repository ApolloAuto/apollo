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
 * @file quartic_polynomial_curve1d.cc
 **/

#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 2>& end,
    const double param)
    : QuarticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                               param) {}

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const double x0, const double dx0, const double ddx0, const double dx1,
    const double ddx1, const double param) {
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_[0] = dx1;
  end_condition_[1] = ddx1;
  ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, param);
}

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const QuarticPolynomialCurve1d& other) {
  param_ = other.param_;
  coef_ = other.coef_;
}

double QuarticPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                          const double p) const {
  switch (order) {
    case 0: {
      return (((coef_[4] * p + coef_[3]) * p + coef_[2]) * p + coef_[1]) * p +
             coef_[0];
    }
    case 1: {
      return ((4.0 * coef_[4] * p + 3.0 * coef_[3]) * p + 2.0 * coef_[2]) * p +
             coef_[1];
    }
    case 2: {
      return (12.0 * coef_[4] * p + 6.0 * coef_[3]) * p + 2.0 * coef_[2];
    }
    case 3: {
      return 24.0 * coef_[4] * p + 6.0 * coef_[3];
    }
    case 4: {
      return 24.0 * coef_[4];
    }
    default:
      return 0.0;
  }
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::FitWithEndPointFirstOrder(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double p) {
  CHECK_GT(p, 0.0);

  param_ = p;

  coef_[0] = x0;

  coef_[1] = dx0;

  coef_[2] = 0.5 * ddx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;

  double b0 = x1 - coef_[0] - coef_[1] * p - coef_[2] * p2;
  double b1 = dx1 - dx0 - ddx0 * p;

  coef_[4] = (b1 * p - 3 * b0) / p4;
  coef_[3] = (4 * b0 - b1 * p) / p3;
  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::FitWithEndPointSecondOrder(
    const double x0, const double dx0, const double x1, const double dx1,
    const double ddx1, const double p) {
  CHECK_GT(p, 0.0);

  param_ = p;

  coef_[0] = x0;

  coef_[1] = dx0;

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;

  double b0 = x1 - coef_[0] - coef_[1] * p;
  double b1 = dx1 - coef_[1];
  double c1 = b1 * p;
  double c2 = ddx1 * p2;

  coef_[2] = (0.5 * c2 - 3 * c1 + 6 * b0) / p2;
  coef_[3] = (-c2 + 5 * c1 - 8 * b0) / p3;
  coef_[4] = (0.5 * c2 - 2 * c1 + 3 * b0) / p4;

  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::IntegratedFromCubicCurve(
    const PolynomialCurve1d& other, const double init_value) {
  CHECK_EQ(other.Order(), 3U);
  param_ = other.ParamLength();
  coef_[0] = init_value;
  for (size_t i = 0; i < 4; ++i) {
    coef_[i + 1] = other.Coef(i) / (static_cast<double>(i) + 1);
  }
  return *this;
}

QuarticPolynomialCurve1d& QuarticPolynomialCurve1d::DerivedFromQuinticCurve(
    const PolynomialCurve1d& other) {
  CHECK_EQ(other.Order(), 5U);
  param_ = other.ParamLength();
  for (size_t i = 1; i < 6; ++i) {
    coef_[i - 1] = other.Coef(i) * static_cast<double>(i);
  }
  return *this;
}

void QuarticPolynomialCurve1d::ComputeCoefficients(
    const double x0, const double dx0, const double ddx0, const double dx1,
    const double ddx1, const double p) {
  CHECK_GT(p, 0.0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;

  double b0 = dx1 - ddx0 * p - dx0;
  double b1 = ddx1 - ddx0;

  double p2 = p * p;
  double p3 = p2 * p;

  coef_[3] = (3 * b0 - b1 * p) / (3 * p2);
  coef_[4] = (-2 * b0 + b1 * p) / (4 * p3);
}

std::string QuarticPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

double QuarticPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(5U, order);
  return coef_[order];
}

}  // namespace planning
}  // namespace apollo
