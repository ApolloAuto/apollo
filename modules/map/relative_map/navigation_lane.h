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

#ifndef MODULES_MAP_RELATIVE_MAP_NAVIGATION_LANE_H_
#define MODULES_MAP_RELATIVE_MAP_NAVIGATION_LANE_H_

#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace relative_map {

class NavigationLane {
 public:
  NavigationLane() = default;
  ~NavigationLane() = default;

  bool Update(const perception::PerceptionObstacles& perception_obstacles);

  const NavigationPath& Path() { return navigation_path_; }

 private:
  double EvaluateCubicPolynomial(const double c0, const double c1,
                                 const double c2, const double c3,
                                 const double z) const;
  void ConvertLaneMarkerToPath(const perception::LaneMarkers& lane_marker,
                               common::Path* path);

  perception::PerceptionObstacles perception_obstacles_;

  NavigationPath navigation_path_;

  common::VehicleState adc_state_;
};

}  // namespace relative_map
}  // namespace apollo

#endif  // MODULES_MAP_RELATIVE_MAP_NAVIGATION_LANE_H_
