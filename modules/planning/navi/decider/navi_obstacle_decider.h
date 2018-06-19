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
 * @brief This file provides the declaration of the class "NaviSpeedDecider".
 */

#ifndef MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_
#define MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_

#include <map>
#include <string>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/navi/common/local_path.h"


/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class NaviObstacleDecider
 * @brief NaviObstacleDecider is used to make appropriate decisions for
 * obstacles around the vehicle in navigation mode.
 * Note that NaviObstacleDecider is only used in navigation mode (turn on
 * navigation mode by setting "FLAGS_use_navigation_mode" to "true") and do not
 * use it in standard mode.
 */
class NaviObstacleDecider {
 public:
  NaviObstacleDecider() = default;

  virtual ~NaviObstacleDecider() = default;

  /**
   * @brief update mobileye's info
   */
  inline void update() { path_obstacle_processed = false; }

  /**
   * @brief process local path's obstacles info
   */
  void ProcessPathObstacle(const std::vector<const Obstacle*>& obstacles,
                           LocalPath* fpath);

  /**
   * @brief get this local path's nudgable distance
   * @return left nudgable distance and right nudgable distance
   */
  void GetLeftRightNudgableDistance(LocalPath* fpath, float* left_nudgable,
                                    float* right_nudgable);

  /**
   * @brief get the actual nudgable distance according to the
   * position of the obstacle
   * @return actual nudgable distance
   */
  float GetNudgeDistance(const float left_nudgable, const float right_nudgable);

  /**
   * @brief Get projection point based on distance
   * @return projection point
   */
  // static Vec2d Interpolate(const float dist, const vector<Vec2d>& path);

  /**
   * @brief
   * @return obstacle's width and distance.
   */
  inline std::map<double, double>& MutableObstacleLatDistance() {
    return obstacle_lat_dist;
  }

 private:
  float front_edge_to_center = 3.89;
  float back_edge_to_center = 1.043;
  float left_edge_to_center = 1.055;
  float right_edge_to_center = 1.055;
  float default_lane_width = 3.3;
  bool path_obstacle_processed = false;
  std::map<double, double> obstacle_lat_dist;

  // TODO(all): Add your member functions and variables.
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_ */
