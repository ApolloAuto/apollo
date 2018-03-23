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

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/map/relative_map/proto/relative_map_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace relative_map {

class NavigationLane {
 public:
  NavigationLane() = default;
  explicit NavigationLane(const NavigationLaneConfig& config);
  ~NavigationLane() = default;

  void SetConfig(const NavigationLaneConfig& config);

  bool GeneratePath();

  void UpdatePerception(
      const perception::PerceptionObstacles& perception_obstacles) {
    perception_obstacles_ = perception_obstacles;
  }

  void UpdateNavigationInfo(const NavigationInfo& navigation_info) {
    navigation_info_ = navigation_info;
    last_project_index_ = 0;
  }

  const NavigationPath& Path() { return navigation_path_; }

  bool CreateMap(const MapGenerationParam& map_config, MapMsg* map_msg) const;

 private:
  double EvaluateCubicPolynomial(const double c0, const double c1,
                                 const double c2, const double c3,
                                 const double z) const;

  double GetKappa(const double c1, const double c2, const double c3,
                  const double x);

  void MergeNavigationLineAndLaneMarker(common::Path* path);

  common::PathPoint GetPathPointByS(const common::Path& path,
                                    const int start_index, const double s,
                                    int* matched_index);

  void ConvertLaneMarkerToPath(const perception::LaneMarkers& lane_marker,
                               common::Path* path);

  void ConvertNavigationLineToPath(common::Path* path);

  bool UpdateProjectionIndex(const common::Path& path);

  NavigationLaneConfig config_;

  // received from topic: /apollo/perception_obstacles
  perception::PerceptionObstacles perception_obstacles_;

  // received from topic: /apollo/navigation
  NavigationInfo navigation_info_;

  // navigation_path_ is the combined results from perception and navigation
  NavigationPath navigation_path_;

  // when invalid, left_width_ < 0
  double left_width_ = -1.0;

  // when invalid, right_width_ < 0
  double right_width_ = -1.0;

  int last_project_index_ = 0;

  // in world coordination: ENU
  localization::Pose original_pose_;
};

}  // namespace relative_map
}  // namespace apollo

#endif  // MODULES_MAP_RELATIVE_MAP_NAVIGATION_LANE_H_
