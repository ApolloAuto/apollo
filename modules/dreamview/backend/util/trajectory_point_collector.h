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
 * @file
 * @brief the class of TrajectoryPointCollector
 */

#ifndef MODULES_DREAMVIEW_BACKEND_UTIL_TRAJECTORY_POINT_COLLECTOR_H_
#define MODULES_DREAMVIEW_BACKEND_UTIL_TRAJECTORY_POINT_COLLECTOR_H_

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/dreamview/proto/simulation_world.pb.h"

/**
 * @namespace apollo::dreamview::util
 * @brief apollo::dreamview::util
 */
namespace apollo {
namespace dreamview {
namespace util {

/**
 * @class TrajectoryPointCollector
 *
 * @brief A utility class that helps simplify the process of adding trajectory
 * points to the SimulationWorld object.
 * The logic is that whenever a new trajectory point is collected, it
 * will create a trajectory segment with its predecessor (except for
 * the first-ever collected trajectory point, obviously, who has no
 * predecessor). The geometry properties of this trajectory segment is
 * then recorded in the SimulationWorld object.
 */
class TrajectoryPointCollector {
 public:
  /**
   * @brief Constructor to start collecting a new planning trajectory.
   * @param world The SimulationWorld object in which the trajectory points will
   * be updated.
   */
  explicit TrajectoryPointCollector(SimulationWorld *world) : world_(world) {
    world_->clear_planning_trajectory();
  }

  /**
   * @brief The Collect method creates a trajectory segment between the input
   * point and the previous point.
   * @param point The trajectory point to be added.
   * @param base_time The timestampe of the first trajectory point
   */
  void Collect(const common::TrajectoryPoint &point, const double base_time);

 private:
  // Does not own the SimulationWorld instance. This is stored as the
  // handle to access and mutate the SimulationWorld object.
  SimulationWorld *world_ = nullptr;

  // Cache (copied) of the previously collected trajectory point. See
  // class documentation for the reason of caching it.
  common::TrajectoryPoint previous_;

  // Indicates whether there has been any collected points.
  bool has_previous_ = false;
};

}  // namespace util
}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_UTIL_TRAJECTORY_POINT_COLLECTOR_H_
