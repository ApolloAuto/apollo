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

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/navi/common/local_path.h"
#include "modules/planning/tasks/task.h"

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
class NaviObstacleDecider : public Task {
 public:
  NaviObstacleDecider();

  virtual ~NaviObstacleDecider() = default;

  /**
   * @brief update mobileye's info
   */
  inline void Update() { path_obstacle_processed_ = false; }

  /**
   * @brief process local path's obstacles info
   */
  void ProcessPathObstacle(const std::vector<const Obstacle*>& obstacles,
                           LocalPath* fpath);

  /**
   * @brief get this local path's nudgable distance
   * @return left nudgable distance and right nudgable distance
   */
  void GetLeftRightNudgableDistance(const double lan_width, LocalPath* fpath,
                                    double* left_nudgable,
                                    double* right_nudgable);

  /**
   * @brief get the actual nudgable distance according to the
   * position of the obstacle
   * @return actual nudgable distance
   */
  double GetNudgeDistance(const double left_nudgable,
                          const double right_nudgable);

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
    return obstacle_lat_dist_;
  }
  /**
   * @brief Get vehicle parameter
   * @return vehicle parameter
   */
  inline const ::apollo::common::VehicleParam& VehicleParam() {
    const auto& vehicle_param = apollo::common::VehicleConfigHelper::instance()
                                    ->GetConfig()
                                    .vehicle_param();
    return vehicle_param;
  }

 private:
  bool path_obstacle_processed_ = false;
  std::map<double, double> obstacle_lat_dist_;

  // TODO(all): Add your member functions and variables.
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_ */
