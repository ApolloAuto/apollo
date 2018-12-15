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

namespace apollo {
namespace planning {

class OpenSpaceROI {
 public:
  explicit OpenSpaceROI(
      const PlannerOpenSpaceConfig &planner_open_space_config);

  bool GenerateRegionOfInterest(Frame *frame);

  const size_t obstacles_num() { return obstacles_num_; }

  const Eigen::MatrixXi &obstacles_edges_num() { return obstacles_edges_num_; }

  const std::vector<std::vector<common::math::Vec2d>>
      &obstacles_vertices_vec() {
    return obstacles_vertices_vec_;
  }

  const Eigen::MatrixXd &obstacles_A() { return obstacles_A_; }

  const Eigen::MatrixXd &obstacles_b() { return obstacles_b_; }

  const double origin_heading() { return origin_heading_; }

  const common::math::Vec2d &origin_point() { return origin_point_; }

  const std::vector<double> &ROI_xy_boundary() { return ROI_xy_boundary_; }

  const std::vector<double> &open_space_end_pose() {
    return open_space_end_pose_;
  }

 private:
  // @brief main process to compute and load info needed by open space planner
  bool GetOpenSpaceInfo();

  // @brief generate the path by vehicle location and return the target parking
  // spot on that path
  bool GetMapInfo(hdmap::ParkingSpaceInfoConstPtr *target_parking_spot,
                  std::shared_ptr<hdmap::Path> *nearby_path);

  // @brief search target parking spot on the path by vehicle location, if
  // no return a nullptr in target_parking_spot
  void SearchTargetParkingSpotOnPath(
      std::shared_ptr<hdmap::Path> *nearby_path,
      hdmap::ParkingSpaceInfoConstPtr *target_parking_spot);

  // @brief if not close enough to parking spot, return false
  bool CheckDistanceToParkingSpot(
      std::shared_ptr<hdmap::Path> *nearby_path,
      hdmap::ParkingSpaceInfoConstPtr *target_parking_spot);

  // @brief "Region of Interest", load open space xy boundary and parking
  // space boundary from pnc map (only for T shape parking space) to
  // ROI_xy_boundary_ and ROI_parking_boundary_
  bool GetOpenSpaceROI();

  // @brief Represent the obstacles in vertices and load it into
  // obstacles_vertices_vec_ in clock wise order. Take different approach
  // towards warm start and distance approach
  bool VPresentationObstacle();

  // @brief Transform the vertice presentation of the obstacles into linear
  // inequality as Ax>b
  bool HPresentationObstacle();

  // @brief Helper function for HPresentationObstacle()
  bool ObsHRep(const size_t &obstacles_num,
               const Eigen::MatrixXi &obstacles_edges_num,
               const std::vector<std::vector<common::math::Vec2d>>
                   &obstacles_vertices_vec,
               Eigen::MatrixXd *A_all, Eigen::MatrixXd *b_all);

 private:
  const hdmap::HDMap *hdmap_ = nullptr;

  ThreadSafeIndexedObstacles *obstacles_by_frame_;

  // @brief obstacles total num including perception obstacles and parking space
  // boundary
  size_t obstacles_num_ = 0;

  common::VehicleState vehicle_state_;

  // @brief the dimension needed for A and b matrix dimension in H
  // representation
  Eigen::MatrixXi obstacles_edges_num_;

  // @brief in the order of [x_min, x_max, y_min, y_max];
  std::vector<double> ROI_xy_boundary_;

  // @brief vectors in the order of left parking bound, parking end bound, right
  // parking bound, opposite lane. And the vertices order is counter-clockwise
  // the viewing angle and the vertice sequence
  //
  //                     8------------------------7   <-  up_boundary
  //
  //  left_boundary  |-> 1-------2      5----------6   <-|  right_boundary
  //                 |->         |      |            <-|
  //                             |      |
  //                             |      |
  //                            |3------4|
  //                                ^
  //                          down_boundary
  // ROI_parking_boundary_ in form of {{1,2,3},{3,4},{4,5,6},{7,8}}
  std::vector<std::vector<common::math::Vec2d>> ROI_parking_boundary_;

  // @brief open_space end configuration in order of x, y, heading and speed.
  // Speed is set to be always zero now for parking
  std::vector<double> open_space_end_pose_;

  // @brief vector storing the vertices of obstacles in counter-clock-wise order
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;

  // @brief Linear inequality representation of the obstacles Ax>b
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;

  // @brief origin heading for planning space rotation
  double origin_heading_ = 0.0;

  // @brief origin point for scaling down the numeric value of the optimization
  // problem in order of x , y
  common::math::Vec2d origin_point_;

  // @brief parking_spot_heading_ is heading the direction pointing away from
  // the lane
  double parking_spot_heading_ = 0.0;

  // @brief parking_spot_id from routing
  std::string target_parking_spot_id_ = "";

  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;

  apollo::common::VehicleParam vehicle_params_;
};

}  // namespace planning
}  // namespace apollo
