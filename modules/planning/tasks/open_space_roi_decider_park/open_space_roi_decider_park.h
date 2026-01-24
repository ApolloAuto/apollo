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

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/tasks/open_space_roi_decider_park/proto/open_space_roi_decider_park.pb.h"

#include "cyber/common/log.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/indexed_queue.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_interface_base/task_base/common/decider.h"

namespace apollo {
namespace planning {
struct ParkingInfo {
  ParkingType parking_type;
  std::string parking_id;
  // left_top right_top right_down left_down
  std::vector<common::math::Vec2d> corner_points;
  common::math::Vec2d center_point;
  bool is_on_left = false;
};

class OpenSpaceRoiDeciderPark : public Decider {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;

 private:
  apollo::common::Status Process(Frame *frame) override;

 private:
  // @brief generate the path by vehicle location and return the target parking
  // spot on that path
  bool GetParkingSpot(Frame *const frame, ParkingInfo *parking_info);

  // @brief get path from reference line and return vertices of pullover spot
  bool GetPullOverSpot(Frame *const frame,
                       std::array<common::math::Vec2d, 4> *vertices,
                       hdmap::Path *nearby_path);

  // @brief Set an origin to normalize the problem for later computation
  void SetOrigin(const ParkingInfo &parking_info, Frame *const frame);
  void SetOrigin(Frame *const frame,
                 const std::array<common::math::Vec2d, 4> &vertices);
  void SetOriginFromADC(Frame *const frame, const hdmap::Path &nearby_path);
  void SetParkingSpotEndPose(const ParkingInfo &parking_info,
                             Frame *const frame);

  void SetPullOverSpotEndPose(Frame *const frame);
  void SetParkAndGoEndPose(Frame *const frame);
  void GetLargeCurvatureEndPose(Frame *const frame, common::math::Vec2d uturn_end_pos, double uturn_heading);

  // @brief Get road boundaries of both sides
  void GetRoadBoundary(
      const hdmap::Path &nearby_path, const double center_line_s,
      const common::math::Vec2d &origin_point, const double origin_heading,
      std::vector<common::math::Vec2d> *left_lane_boundary,
      std::vector<common::math::Vec2d> *right_lane_boundary,
      std::vector<common::math::Vec2d> *center_lane_boundary_left,
      std::vector<common::math::Vec2d> *center_lane_boundary_right,
      std::vector<double> *center_lane_s_left,
      std::vector<double> *center_lane_s_right,
      std::vector<double> *left_lane_road_width,
      std::vector<double> *right_lane_road_width);

  // @brief Get the Road Boundary From Map object
  void GetRoadBoundaryFromMap(
      const hdmap::Path &nearby_path, const double center_line_s,
      const common::math::Vec2d &origin_point, const double origin_heading,
      std::vector<common::math::Vec2d> *left_lane_boundary,
      std::vector<common::math::Vec2d> *right_lane_boundary,
      std::vector<common::math::Vec2d> *center_lane_boundary_left,
      std::vector<common::math::Vec2d> *center_lane_boundary_right,
      std::vector<double> *center_lane_s_left,
      std::vector<double> *center_lane_s_right,
      std::vector<double> *left_lane_road_width,
      std::vector<double> *right_lane_road_width);

  // @brief Check single-side curb and add key points to the boundary
  void AddBoundaryKeyPoint(
      const hdmap::Path &nearby_path, const double check_point_s,
      const double start_s, const double end_s, const bool is_anchor_point,
      const bool is_left_curb,
      std::vector<common::math::Vec2d> *center_lane_boundary,
      std::vector<common::math::Vec2d> *curb_lane_boundary,
      std::vector<double> *center_lane_s, std::vector<double> *road_width);

  // @brief "Region of Interest", load map boundary for open space scenario
  // @param vertices is an array consisting four points describing the
  // boundary of spot in box. Four points are in sequence of left_top,
  // left_down, right_down, right_top
  // ------------------------------------------------------------------
  //
  //                     --> lane_direction
  //
  // ----------------left_top        right_top--------------------------
  //                -                  -
  //                -                  -
  //                -                  -
  //                -                  -
  //                left_down-------right_down
  bool GetParkingBoundary(const ParkingInfo &parking_info,
                          const hdmap::Path &nearby_path, Frame *const frame,
                          std::vector<std::vector<common::math::Vec2d>>
                              *const roi_parking_boundary,
                          std::vector<std::vector<common::math::Vec2d>>
                              *const parking_soft_boundary);

  bool GetParkingOutBoundary(const hdmap::Path &nearby_path, Frame *const frame,
                             std::vector<std::vector<common::math::Vec2d>>
                                 *const roi_parking_boundary);

  bool GetPullOverBoundary(Frame *const frame,
                           const std::array<common::math::Vec2d, 4> &vertices,
                           const hdmap::Path &nearby_path,
                           std::vector<std::vector<common::math::Vec2d>>
                               *const roi_parking_boundary);

  bool GetParkAndGoBoundary(Frame *const frame, const hdmap::Path &nearby_path,
                            std::vector<std::vector<common::math::Vec2d>>
                                *const roi_parking_boundary);

  bool GetLargeCurvatureBoundary(Frame *const frame, const hdmap::Path &nearby_path, std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary);

  // @brief search target parking spot on the path by vehicle location, if
  // no return a nullptr in target_parking_spot
  void SearchTargetParkingSpotOnPath(
      const hdmap::Path &nearby_path,
      hdmap::ParkingSpaceInfoConstPtr *target_parking_spot);

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
  /**
   * @brief check if vehicle is parked in a parking lot
   *
   * @return true adc parked in a parking lot
   * @return false adc parked at a pull-over spot
   */
  bool IsInParkingLot(const double adc_init_x, const double adc_init_y,
                      const double adc_init_heading,
                      std::array<common::math::Vec2d, 4> *parking_lot_vertices);
  /**
   * @brief Get the Park Spot From Map object
   *
   * @param parking_lot
   * @param vertices
   */
  void GetParkSpotFromMap(hdmap::ParkingSpaceInfoConstPtr parking_lot,
                          std::array<common::math::Vec2d, 4> *vertices);

  /**
   * @brief Collect all the lane segments in the routing reponse.
   *
   * @param routing_response The routing response containing the lane segments.
   * @param routing_segments The output vector of lane segments.
   */
  void GetAllLaneSegments(const routing::RoutingResponse &routing_response,
                          std::vector<routing::LaneSegment> *routing_segments);

  bool GetNearbyPath(const apollo::routing::RoutingResponse &routing_response,
                     const hdmap::ParkingSpaceInfoConstPtr &parking_spot,
                     std::shared_ptr<hdmap::Path> *nearby_path);
  
  bool GetUTurnPath(Frame *const frame, std::shared_ptr<hdmap::Path> *nearby_path);

  bool AdjustPointsOrderToClockwise(std::vector<common::math::Vec2d> *polygon);

  bool AddParkingSpaceBoundary(Frame *const frame,
                               const hdmap::Path &nearby_path,
                               std::vector<std::vector<common::math::Vec2d>>
                                   *const roi_parking_boundary);

 private:
  void GetParkingSoftBoundary(
    common::math::Vec2d left_top, common::math::Vec2d left_down,
    common::math::Vec2d right_top, common::math::Vec2d right_down,
    std::vector<std::vector<common::math::Vec2d>> *const parking_soft_boundary);

  // @brief parking_spot_id from routing
  std::string target_parking_spot_id_;
  std::shared_ptr<apollo::hdmap::Path> nearby_path_;
  const hdmap::HDMap *hdmap_ = nullptr;

  apollo::common::VehicleParam vehicle_params_;

  ThreadSafeIndexedObstacles *obstacles_by_frame_;

  common::VehicleState vehicle_state_;
  OpenSpaceRoiDeciderParkConfig config_;
  bool is_parking_out = false;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::OpenSpaceRoiDeciderPark,
                                     Task)

}  // namespace planning
}  // namespace apollo
