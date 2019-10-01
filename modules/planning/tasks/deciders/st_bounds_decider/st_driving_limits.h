/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 *   @file
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/speed_bounds_decider_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class STDrivingLimits {
 public:
  STDrivingLimits();

  virtual ~STDrivingLimits() = default;

  common::Status ComputeSTDrivingLimits();

  std::pair<double, double> GetVehicleDynamicsLimits(double t) const;

 private:
  double t_resolution_;
  // The limits due to vehicle dynamics (expressed as s vs. t), such as
  // max. acceleration, max. cruise speed, etc.
  std::vector<std::tuple<double, double, double>> vehicle_dynamics_limits_t_s_;

  // The limits expressed as v vs. s, which contains the following parts:
  //  1. speed limits at path segments with big curvatures.
  std::vector<std::tuple<double, double, double>> curvature_speed_limits_s_v_;
  //  2. speed limits from traffic limits (speed bumps, etc.).
  std::vector<std::tuple<double, double, double>> traffic_speed_limits_s_v_;
  //  3. speed limits for safety considerations when other obstacles are nearby
  std::vector<std::tuple<double, double, double>> obstacles_speed_limits_s_v_;
};
