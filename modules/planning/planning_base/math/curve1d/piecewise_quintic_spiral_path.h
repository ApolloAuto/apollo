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

#pragma once

#include <string>
#include <vector>

#include "modules/planning/planning_base/math/curve1d/curve1d.h"
#include "modules/planning/planning_base/math/curve1d/quintic_spiral_path.h"

namespace apollo {
namespace planning {

class PiecewiseQuinticSpiralPath : public Curve1d {
 public:
  PiecewiseQuinticSpiralPath(const double theta, const double kappa,
                             const double dkappa);

  virtual ~PiecewiseQuinticSpiralPath() = default;

  void Append(const double theta, const double kappa, const double dkappa,
              const double delta_s);

  double Evaluate(const std::uint32_t order, const double param) const override;

  double DeriveKappaS(const double s) const;

  double ParamLength() const override;

  std::string ToString() const override { return "PiecewiseQuinticSpiralPath"; }

 private:
  size_t LocatePiece(const double param) const;

  std::vector<QuinticSpiralPath> pieces_;

  std::vector<double> accumulated_s_;

  double last_theta_ = 0.0;

  double last_kappa_ = 0.0;

  double last_dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
