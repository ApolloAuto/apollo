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
 * @brief: Implementation of PiecewiseLinearConstraint class
 **/

#include "modules/planning/math/smoothing_spline/piecewise_linear_constraint.h"

#include <limits>

namespace apollo {
namespace planning {

PiecewiseLinearConstraint::PiecewiseLinearConstraint(const uint32_t dimension)
    : dimension_(dimension) {}

bool PiecewiseLinearConstraint::AddBoundary(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddDerivativeBoundary(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddSecondDerivativeBoundary(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddThirdDerivativeBoundary(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointConstraint(const double x,
                                                   const double fx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointDerivativeConstraint(const double x,
                                                             const double dfx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointSecondDerivativeConstraint(
    const double x, const double ddfx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointThirdDerivativeConstraint(
    const double x, const double dddfx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddMonotoneInequalityConstraint() {
  // TODO(Liangliang): implement this function
  return true;
}

}  // namespace planning
}  // namespace apollo
