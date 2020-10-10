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
 * @file
 **/

#include "modules/planning/math/curve1d/cubic_polynomial_curve1d.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
    const std::array<double, 3>& start, const double end, const double param)
    : CubicPolynomialCurve1d(start[0], start[1], start[2], end, param) {}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(const double x0,
                                               const double dx0,
                                               const double ddx0,
                                               const double x1,
                                               const double param) {
  ComputeCoefficients(x0, dx0, ddx0, x1, param);
  param_ = param;
  start_condition_[0] = x0;
  start_condition_[1] = dx0;
  start_condition_[2] = ddx0;
  end_condition_ = x1;
}

void CubicPolynomialCurve1d::DerivedFromQuarticCurve(
    const PolynomialCurve1d& other) {
  CHECK_EQ(other.Order(), 4);
  param_ = other.ParamLength();
  for (size_t i = 1; i < 5; ++i) {
    coef_[i - 1] = other.Coef(i) * static_cast<double>(i);
  }
}

double CubicPolynomialCurve1d::Evaluate(const std::uint32_t order,
                                        const double p) const {
  switch (order) {
    case 0: {
      return ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
    }
    case 1: {
      return (3.0 * coef_[3] * p + 2.0 * coef_[2]) * p + coef_[1];
    }
    case 2: {
      return 6.0 * coef_[3] * p + 2.0 * coef_[2];
    }
    case 3: {
      return 6.0 * coef_[3];
    }
    default:
      return 0.0;
  }
}

std::string CubicPolynomialCurve1d::ToString() const {
  return absl::StrCat(absl::StrJoin(coef_, "\t"), param_, "\n");
}

void CubicPolynomialCurve1d::ComputeCoefficients(const double x0,
                                                 const double dx0,
                                                 const double ddx0,
                                                 const double x1,
                                                 const double param) {
  DCHECK(param > 0.0);
  const double p2 = param * param;
  const double p3 = param * p2;
  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5 * ddx0;
  coef_[3] = (x1 - x0 - dx0 * param - coef_[2] * p2) / p3;
}

double CubicPolynomialCurve1d::Coef(const size_t order) const {
  CHECK_GT(4, order);
  return coef_[order];
}

}  // namespace planning
}  // namespace apollo
