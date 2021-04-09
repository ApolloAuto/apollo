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
 * @file: spiral_curve.cc
 **/
#include "modules/planning/math/spiral_curve/spiral_curve.h"

#include <cmath>
#include <limits>

namespace apollo {
namespace planning {

using apollo::common::PathPoint;

SpiralCurve::SpiralCurve(const PathPoint& s, const PathPoint& e,
                         const std::uint32_t order)
    : start_point_(&s),
      end_point_(&e),
      p_params_(order + 1, 0.0),
      sg_(0.0),
      error_(std::numeric_limits<double>::infinity()) {}

void SpiralCurve::SetSpiralConfig(const SpiralCurveConfig& spiral_config) {
  spiral_config_ = spiral_config;
}

// output params
const PathPoint& SpiralCurve::start_point() const { return *start_point_; }

const PathPoint& SpiralCurve::end_point() const { return *end_point_; }

double SpiralCurve::sg() const { return sg_; }

double SpiralCurve::error() const { return error_; }

const std::vector<double>& SpiralCurve::p_params() const { return p_params_; }
const SpiralCurveConfig& SpiralCurve::spiral_config() const {
  return spiral_config_;
}

void SpiralCurve::set_sg(const double sg) { sg_ = sg; }

void SpiralCurve::set_error(const double error) { error_ = error; }

bool SpiralCurve::ResultSanityCheck() const {
  for (const auto& p : p_params_) {
    if (std::isnan(p)) {
      return false;
    }
  }
  return (sg_ > 0);
}
}  // namespace planning
}  // namespace apollo
