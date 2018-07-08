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

#ifndef MODULES_PLANNING_NAVI_NAVI_SPEED_TS_GRAPH_H_
#define MODULES_PLANNING_NAVI_NAVI_SPEED_TS_GRAPH_H_

#include <functional>
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
  // The minimum timestamp of the point, can't less than it.
  double t_min = 0.0;
  // The maximum speed of the point, can't lgreater than it.
  double v_max = std::numeric_limits<double>::max();
  // The preffered speed of the point.
  double v_preffered = std::numeric_limits<double>::max();
  // The maximum acceleration of the point, can't lgreater than it.
  double a_max = std::numeric_limits<double>::max();
  // The preffered acceleration of the point.
  double a_preffered = std::numeric_limits<double>::max();
  // The maximum deceleration of the point, can't lgreater than it.
  double b_max = std::numeric_limits<double>::max();
  // The preffered deceleration of the point.
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
 * and preffered.
 */
class NaviSpeedTsGraph {
 public:
  NaviSpeedTsGraph();

  /**
   * @brief Reset points's num and time step etc.
   * @param s_step S step between two points.
   * @param s_max Max of s-axis.
   * @param get_safe_distance Callback for get safe distance base on v.
   */
  void Reset(double s_step, double s_max,
             const std::function<double(double v)>& get_safe_distance);

  /**
   * @brief Get the current s-step.
   * @return Current s-step.
   */
  double Step() const;

  /**
   * @brief Get the current num of s-points.
   * @return Current num of s-points.
   */
  std::size_t PointNum() const;

  /**
   * @brief Assign constraints to all points.
   * @param constraints constraints for all points.
   */
  void UpdateConstraints(const NaviSpeedTsConstraints& constraints);

  /**
   * @brief Assign constraints to one point.
   * @param s S fo the point.
   * @param constraints constraints for the point.
   */
  void UpdatePointConstraints(double s,
                              const NaviSpeedTsConstraints& constraints);

  /**
   * @brief Assign constraints for an obstacle.
   * @param distance The distance from vehicle to the obstacle.
   * @param v Speed of the obstacle.
   * @param constraints constraints for the point.
   */
  void UpdateObstacleConstraints(double distance, double v);

  /**
   * @brief Solving the t-s curve.
   * @param start_v V of the start point.
   * @param start_a A of the start point.
   * @param start_da Da of the start point.
   * @param points buffer of t-s points for output.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Solve(double start_v, double start_a, double start_da,
                               std::vector<NaviSpeedTsPoint>* points);

 private:
  std::vector<NaviSpeedTsConstraints> constraints_;
  double s_step_;
  std::function<double(double)> get_safe_distance_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_NAVI_SPEED_TS_GRAPH_H_
