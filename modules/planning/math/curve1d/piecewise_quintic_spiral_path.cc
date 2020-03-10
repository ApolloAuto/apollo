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

/**
 * @file
 **/

#include "modules/planning/math/curve1d/piecewise_quintic_spiral_path.h"

namespace apollo {
namespace planning {

PiecewiseQuinticSpiralPath::PiecewiseQuinticSpiralPath(const double theta,
                                                       const double kappa,
                                                       const double dkappa)
    : last_theta_(theta), last_kappa_(kappa), last_dkappa_(dkappa) {
  accumulated_s_.push_back(0.0);
}

void PiecewiseQuinticSpiralPath::Append(const double theta, const double kappa,
                                        const double dkappa,
                                        const double delta_s) {
  double s = delta_s + accumulated_s_.back();
  accumulated_s_.push_back(s);

  pieces_.emplace_back(last_theta_, last_kappa_, last_dkappa_, theta, kappa,
                       dkappa, delta_s);

  last_theta_ = theta;
  last_kappa_ = kappa;
  last_dkappa_ = dkappa;
}

double PiecewiseQuinticSpiralPath::Evaluate(const std::uint32_t order,
                                            const double s) const {
  auto index = LocatePiece(s);
  return pieces_[index].Evaluate(order, s - accumulated_s_[index]);
}

double PiecewiseQuinticSpiralPath::DeriveKappaS(const double s) const {
  auto index = LocatePiece(s);
  const auto& piece = pieces_[index];
  double ratio = (s - accumulated_s_[index]) / piece.ParamLength();
  return piece.DeriveKappaDerivative(QuinticSpiralPath::DELTA_S, ratio);
}

double PiecewiseQuinticSpiralPath::ParamLength() const {
  return accumulated_s_.back();
}

size_t PiecewiseQuinticSpiralPath::LocatePiece(const double s) const {
  ACHECK(s >= accumulated_s_.front() && s <= accumulated_s_.back());

  auto it_lower =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);

  if (it_lower == accumulated_s_.begin()) {
    return 0;
  } else {
    return std::distance(accumulated_s_.begin(), it_lower) - 1;
  }
}

}  // namespace planning
}  // namespace apollo
