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
 * @file lattice_params.h
 **/

#ifndef MODULES_PLANNING_LATTICE_LATTICE_PARAMS_H_
#define MODULES_PLANNING_LATTICE_LATTICE_PARAMS_H_

namespace apollo {
namespace planning {

// TODO(all) move the fake FLAG variable to planning_gflags
static const double speed_limit = 15.0;
static const double planned_trajectory_time = 8.0;
static const double planned_trajectory_horizon = 200.0;
static const double stop_margin = 2.0;
static const double stop_speed_threshold = 0.1;
static const double low_speed_threshold = 2.0;
static const double enable_stop_handling = true;
static const double trajectory_time_resolution = 0.05;
static const double stop_acc_thred = 1.0;
static const double dist_thred_omit_routing_end = 1.0;
static const double lateral_enter_lane_thred = 2.0;
static const double max_acceleration = 4.0;
static const double min_acceleration = -4.0;
static const double max_deceleration = -min_acceleration;

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_LATTICE_LATTICE_PARAMS_H_ */
