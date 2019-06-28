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
 * @brief This file provides the declaration of the class "NaviSpeedTsGraph".
 */

#pragma once

#include <limits>
#include <vector>

#include "modules/common/status/status.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @struct NaviSpeedTsConstraints
 * @brief NaviSpeedTsConstraints is used to describe constraints of a t-s point.
 */
struct NaviSpeedTsConstraints {
  // The minimum timestamp of the point.
  double t_min = 0.0;
  // The maximum speed of the point.
  double v_max = std::numeric_limits<double>::max();
  // The preferred speed of the point.
  double v_preffered = std::numeric_limits<double>::max();
  // The maximum acceleration of the point.
  double a_max = std::numeric_limits<double>::max();
  // The preferred acceleration of the point.
  double a_preffered = std::numeric_limits<double>::max();
  // The maximum deceleration of the point.
  double b_max = std::numeric_limits<double>::max();
  // The preferred deceleration of the point.
  double b_preffered = std::numeric_limits<double>::max();
  // TODO(all): ignore
  double da_max = std::numeric_limits<double>::max();
  // TODO(all): ignore
  double da_preffered = std::numeric_limits<double>::max();
};

/**
 * @struct NaviSpeedTsPoint
 * @brief NaviSpeedTsPoint is used to describe a t-s point.
 */
struct NaviSpeedTsPoint {
  double s;
  double t;
  double v;
  double a;
  double da;
};

/**
 * @class NaviSpeedTsGraph
 * @brief NaviSpeedTsGraph is used to generate a t-s graph with some limits
 * and preferred.
 */
class NaviSpeedTsGraph {
 public:
  NaviSpeedTsGraph();

  /**
   * @brief Reset points's num and time step etc.
   * @param s_step S step between two points.
   * @param s_max Max of s-axis.
   * @param start_v V of the start point.
   * @param start_a A of the start point.
   * @param start_da Da of the start point.
   */
  void Reset(double s_step, double s_max, double start_v, double start_a,
             double start_da);

  /**
   * @brief Assign constraints to all points.
   * @param constraints constraints for all points.
   */
  void UpdateConstraints(const NaviSpeedTsConstraints& constraints);

  /**
   * @brief Assign constraints to a range.
   * @param start_s S fo the start point.
   * @param end_s S fo the end point.
   * @param constraints constraints for points.
   */
  void UpdateRangeConstraints(double start_s, double end_s,
                              const NaviSpeedTsConstraints& constraints);

  /**
   * @brief Assign constraints for an obstacle.
   * @param distance The distance from vehicle to the obstacle.
   * @param safe_distance The safe distance from vehicle to the obstacle.
   * @param following_accel_ratio Decide the level of acceleration when
   * following obstacle.
   * @param v Speed of the obstacle.
   * @param cruise_speed Cruise speed of vehicle.
   * @param constraints constraints for the point.
   */
  void UpdateObstacleConstraints(double distance, double safe_distance,
                                 double following_accel_ratio, double v,
                                 double cruise_speed);

  /**
   * @brief Solving the t-s curve.
   * @param points buffer of t-s points for output.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Solve(std::vector<NaviSpeedTsPoint>* points);

 private:
  std::vector<NaviSpeedTsConstraints> constraints_;
  double s_step_;
  double start_v_;
  double start_a_;
  double start_da_;
};

}  // namespace planning
}  // namespace apollo
