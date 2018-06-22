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
 * @file trajectory.h
 **/

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class Trajectory {
 public:
  Trajectory() = default;

  virtual ~Trajectory() = default;

  virtual common::TrajectoryPoint Evaluate(
      const double relative_time) const = 0;

  virtual common::TrajectoryPoint StartPoint() const = 0;

  virtual double GetTemporalLength() const = 0;

  virtual double GetSpatialLength() const = 0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
