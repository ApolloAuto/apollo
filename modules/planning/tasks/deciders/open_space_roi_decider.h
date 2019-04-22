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
 * @file
 **/

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "cyber/common/log.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {
class OpenSpaceRoiDecider : public Decider {
 public:
  explicit OpenSpaceRoiDecider(const TaskConfig &config);

 private:
  apollo::common::Status Process(Frame *frame) override;

 private:
  // @brief "Region of Interest", load map boundary for parking scenario
  bool GetParkingBoundary(Frame *const frame,
                          std::vector<std::vector<common::math::Vec2d>>
                              *const roi_parking_boundary);

  // @brief generate the path by vehicle location and return the target parking
  // spot on that path
  bool GetParkingSpotFromMap(
      Frame *const frame, hdmap::ParkingSpaceInfoConstPtr *target_parking_spot,
      std::shared_ptr<hdmap::Path> *nearby_path);

  // @brief search target parking spot on the path by vehicle location, if
  // no return a nullptr in target_parking_spot
  void SearchTargetParkingSpotOnPath(
      const hdmap::Path &nearby_path,
      hdmap::ParkingSpaceInfoConstPtr *target_parking_spot);

  // @brief if not close enough to parking spot, return false
  bool CheckDistanceToParkingSpot(
      const hdmap::Path &nearby_path,
      const hdmap::ParkingSpaceInfoConstPtr &target_parking_spot);

  // @brief Helper function for fuse line segments into convex vertices set
  bool FuseLineSegments(
      std::vector<std::vector<common::math::Vec2d>> *line_segments_vec);

  // @brief main process to compute and load info needed by open space planner
  bool FormulateBoundaryConstraints(
      const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
      Frame *const frame);

  // @brief Represent the obstacles in vertices and load it into
  // obstacles_vertices_vec_ in clock wise order. Take different approach
  // towards warm start and distance approach
  bool LoadObstacleInVertices(
      const std::vector<std::vector<common::math::Vec2d>> &roi_parking_boundary,
      Frame *const frame);

  bool FilterOutObstacle(const Frame &frame, const Obstacle &obstacle);

  // @brief Transform the vertice presentation of the obstacles into linear
  // inequality as Ax>b
  bool LoadObstacleInHyperPlanes(Frame *const frame);

  // @brief Helper function for LoadObstacleInHyperPlanes()
  bool GetHyperPlanes(const size_t &obstacles_num,
                      const Eigen::MatrixXi &obstacles_edges_num,
                      const std::vector<std::vector<common::math::Vec2d>>
                          &obstacles_vertices_vec,
                      Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);

 private:
  // @brief parking_spot_id from routing
  std::string target_parking_spot_id_;

  const hdmap::HDMap *hdmap_ = nullptr;

  apollo::common::VehicleParam vehicle_params_;

  ThreadSafeIndexedObstacles *obstacles_by_frame_;

  common::VehicleState vehicle_state_;
};

}  // namespace planning
}  // namespace apollo
