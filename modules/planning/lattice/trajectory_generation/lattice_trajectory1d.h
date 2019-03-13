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

#include <memory>
#include <string>

#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class LatticeTrajectory1d : public Curve1d {
 public:
  explicit LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d);

  virtual ~LatticeTrajectory1d() = default;

  virtual double Evaluate(const std::uint32_t order, const double param) const;

  virtual double ParamLength() const;

  virtual std::string ToString() const;

  bool has_target_position() const;

  bool has_target_velocity() const;

  bool has_target_time() const;

  double target_position() const;

  double target_velocity() const;

  double target_time() const;

  void set_target_position(double target_position);

  void set_target_velocity(double target_velocity);

  void set_target_time(double target_time);

 private:
  std::shared_ptr<Curve1d> ptr_trajectory1d_;

  double target_position_ = 0.0;

  double target_velocity_ = 0.0;

  double target_time_ = 0.0;

  bool has_target_position_ = false;

  bool has_target_velocity_ = false;

  bool has_target_time_ = false;
};

}  // namespace planning
}  // namespace apollo
