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
 * @file trajectory1d_info.h
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY1D_INFO_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY1D_INFO_H_

#include <memory>

#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

using Trajectory1d = Curve1d;

class Trajectory1dInfo {
 public:
  enum class Type {
    QUINTIC_POLYNOMIAL,
    QUARTIC_POLYNOMIAL,
    CONSTANT_DECELERATION,
    PIECEWISE_ACCELERATION,
    STANDING_STILL
  };

  Trajectory1dInfo();

  virtual ~Trajectory1dInfo() = default;

  void set_type(const Type&);

  std::shared_ptr<Trajectory1d> create() const;

 private:
  Type type_;
};

} // namespace planning
} // namespace apollo

#endif /* MODULES_PLANNING_LATTICE_TRAJECTORY1D_INFO_H_ */
