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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace prediction {

class PredictionMap {
 public:
  /**
   * @brief Check if map is ready
   * @return True if map is ready
   */
  static bool Ready();

  /**
   * @brief Get the position of a point on a specific distance along a lane.
   * @param lane_info The lane to get a position.
   * @param s The distance along the lane.
   * @return The position with coordinates.
   */
  static Eigen::Vector2d PositionOnLane(
      const std::shared_ptr<const hdmap::LaneInfo> lane_info, const double s);

  /**
   * @brief Get the heading of a point on a specific distance along a lane.
   * @param lane_info The lane to get a heading.
   * @param s The distance along the lane.
   * @return The heading of the point.
   */
  static double HeadingOnLane(
      const std::shared_ptr<const hdmap::LaneInfo> lane_info, const double s);

  /**
   * @brief Get the curvature of a point on a specific distance along a lane.
   * @param lane_iid The id of the lane to get a curvature.
   * @param s The distance along the lane.
   * @return The curvature of the point.
   */
  static double CurvatureOnLane(const std::string& lane_id, const double s);

  /**
   * @brief Get the width on a specified distance on a lane.
   * @param lane_info The lane to get the width.
   * @param s The distance along the lane.
   * @return The width on the distance s.
   */
  static double LaneTotalWidth(
      const std::shared_ptr<const hdmap::LaneInfo> lane_info, const double s);

  /**
   * @brief Get a shared pointer to a lane by lane ID.
   * @param id The ID of the target lane ID in the form of string.
   * @return A shared pointer to the lane with the input lane ID.
   */
  static std::shared_ptr<const hdmap::LaneInfo> LaneById(const std::string& id);

  /**
   * @brief Get a shared pointer to a junction by junction ID.
   * @param id The ID of the target junction ID in the form of string.
   * @return A shared pointer to the junction with the input junction ID.
   */
  static std::shared_ptr<const hdmap::JunctionInfo> JunctionById(
      const std::string& id);

  /**
   * @brief Get a shared pointer to an overlap by overlap ID.
   * @param id The ID of the target overlap ID in the form of string.
   * @return A shared pointer to the overlap with the input overlap ID.
   */
  static std::shared_ptr<const hdmap::OverlapInfo> OverlapById(
      const std::string& id);

  /**
   * @brief Get the frenet coordinates (s, l) on a lane by a position.
   * @param position The position to get its frenet coordinates.
   * @param lane_info The lane on which to get the frenet coordinates.
   * @param s The longitudinal coordinate of the position.
   * @param l The lateral coordinate of the position.
   */
  static bool GetProjection(
      const Eigen::Vector2d& position,
      const std::shared_ptr<const hdmap::LaneInfo> lane_info, double* s,
      double* l);

  /**
   * @brief If there is a lane in the range with radius
   * @param x x-axis coordinate
   * @param y y-axis coordinate
   * @param radius range radius
   * @return If there is a lane in the range with radius
   */
  static bool HasNearbyLane(const double x, const double y,
                            const double radius);

  /**
   * @brief Get the nearest path point to a longitudinal coordinate on a lane.
   * @param lane_info The lane on which to get the projected point.
   * @param s The longitudinal coordinate.
   * @param path_point The nearest path point.
   * @param If the process is successful.
   */
  static bool ProjectionFromLane(
      std::shared_ptr<const hdmap::LaneInfo> lane_info, const double s,
      hdmap::MapPathPoint* path_point);

  /**
   * @brief Determine if a lane is a virtual lane.
   * @param The lane ID of the lane.
   * @return If the lane is a virtual lane.
   */
  static bool IsVirtualLane(const std::string& lane_id);

  /**
   * @brief Determine if a point is on a virtual lane.
   * @param The point coordinate.
   * @return If the point is on a virtual lane.
   */
  static bool OnVirtualLane(const Eigen::Vector2d& position,
                            const double radius);

  /**
   * @brief Get the connected lanes from some specified lanes.
   * @param prev_lanes The lanes from which to search their connected lanes.
   * @param heading The specified heading.
   * @param radius The searching radius.
   * @param on_lane If the position is on lane.
   * @param lanes The searched lanes.
   */
  static void OnLane(
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& prev_lanes,
      const Eigen::Vector2d& point, const double heading, const double radius,
      const bool on_lane, const int max_num_lane,
      const double max_lane_angle_diff,
      std::vector<std::shared_ptr<const hdmap::LaneInfo>>* lanes);

  /**
   * @brief Get the lane that the position is on with minimal angle diff
   * @param position the position of an obstacle
   * @param radius The searching radius
   * @param heading The specified heading
   * @param angle_diff_threshold Threshold of angle diff
   * @return A pointer to a lane info
   */
  static std::shared_ptr<const apollo::hdmap::LaneInfo>
  GetMostLikelyCurrentLane(const common::PointENU& position,
                           const double radius, const double heading,
                           const double angle_diff_threshold);

  /**
   * @brief Check whether the projection of ego vehicle is on the target lane.
   *        Note: the direction of the lane is approximated
   *              by the terminal points.
   * @param ego_position The position of the ego vehicle
   * @param lane_id The target lane to project
   * @return true if the projection of ego vehicle is within the lane range;
   *         false otherwise.
   */
  static bool IsProjectionApproximateWithinLane(
      const Eigen::Vector2d& ego_position, const std::string& lane_id);

  /**
   * @brief Check if there are any junctions within the range centered at
   *        a certain point with a radius.
   * @param point The position.
   * @param radius The radius to search junctions.
   * @return If any junctions exist.
   */
  static bool NearJunction(const Eigen::Vector2d& point, const double radius);

  /**
   * @brief Check if a point with coord x and y is in the junction.
   * @param X axis coordinate.
   * @param Y axis coordinate.
   * @param A pointer to junction info.
   * @return If the point is in the junction.
   */
  static bool IsPointInJunction(
      const double x, const double y,
      const std::shared_ptr<const hdmap::JunctionInfo> junction_info_ptr);

  /**
   * @brief Check if the obstacle is in a junction.
   * @param point position
   * @param radius the radius to search candidate junctions
   * @return If the obstacle is in a junction.
   */
  static bool InJunction(const Eigen::Vector2d& point, const double radius);

  /**
   * @brief Check if a lane is in a junction
   * @param lane
   * @param junction id
   * @return If the lane is in the junction
   */
  static bool IsLaneInJunction(
      const std::shared_ptr<const hdmap::LaneInfo> lane_info,
      const std::string& junction_id);

  /**
   * @brief Get a list of junctions given a point and a search radius
   * @param Point
   * @param Search radius
   * @return A list of junctions
   */
  static std::vector<std::shared_ptr<const apollo::hdmap::JunctionInfo>>
  GetJunctions(const Eigen::Vector2d& point, const double radius);

  /**
   * @brief Get a list of junctions given a point and a search radius
   * @param Point
   * @param Search radius
   * @return A list of junctions
   */
  static std::vector<std::shared_ptr<const apollo::hdmap::PNCJunctionInfo>>
  GetPNCJunctions(const Eigen::Vector2d& point, const double radius);

  /**
   * @brief Get the lane heading on a point.
   * @param lane_info The target lane.
   * @param point The point to get the heading.
   * @return The heading of the input point on the input lane.
   */
  static double PathHeading(std::shared_ptr<const hdmap::LaneInfo> lane_info,
                            const common::PointENU& point);

  /**
   * @brief Get the smooth point on a lane by a longitudinal coordinate.
   * @param id The lane ID.
   * @param s The longitudinal coordinate along the lane.
   * @param l The lateral coordinate of the position.
   * @param point The point corresponding to the s,l-value coordinate.
   * @param heading The lane heading on the point.
   * @return If the process is successful.
   */
  static bool SmoothPointFromLane(const std::string& id, const double s,
                                  const double l, Eigen::Vector2d* point,
                                  double* heading);

  /**
   * @brief Get nearby lanes by a position and current lanes.
   * @param point The position to search its nearby lanes.
   * @param heading The heading of an obstacle.
   * @param radius The searching radius.
   * @param lanes The current lanes.
   * @param nearby_lanes The searched nearby lanes.
   */
  static void NearbyLanesByCurrentLanes(
      const Eigen::Vector2d& point, const double heading, const double radius,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes,
      const int max_num_lane,
      std::vector<std::shared_ptr<const hdmap::LaneInfo>>* nearby_lanes);

  static std::shared_ptr<const hdmap::LaneInfo> GetLeftNeighborLane(
      const std::shared_ptr<const hdmap::LaneInfo>& ptr_ego_lane,
      const Eigen::Vector2d& ego_position, const double threshold);

  static std::shared_ptr<const hdmap::LaneInfo> GetRightNeighborLane(
      const std::shared_ptr<const hdmap::LaneInfo>& ptr_ego_lane,
      const Eigen::Vector2d& ego_position, const double threshold);

  /**
   * @brief Get nearby lanes by a position.
   * @param point The position to search its nearby lanes.
   * @param radius The searching radius.
   * @return A vector of nearby lane IDs.
   */
  static std::vector<std::string> NearbyLaneIds(const Eigen::Vector2d& point,
                                                const double radius);

  /**
   * @brief Check if a lane is a left neighbor of another lane.
   * @param target_lane The lane to check if it is a left neighbor.
   * @param curr_lane The current lane.
   * @return If left_lane is a left neighbor of curr_lane.
   */
  static bool IsLeftNeighborLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      std::shared_ptr<const hdmap::LaneInfo> curr_lane);

  /**
   * @brief Check if the target lane is a left neighbor of one of some lanes.
   * @param target_lane The lane to check if it is a left neighbor.
   * @param lanes The current lanes.
   * @return If left_lane is a left neighbor of one of lanes.
   */
  static bool IsLeftNeighborLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes);

  /**
   * @brief Check if a lane is a right neighbor of another lane.
   * @param target_lane The lane to check if it is a right neighbor.
   * @param curr_lane The current lane.
   * @return If right_lane is a right neighbor of curr_lane.
   */
  static bool IsRightNeighborLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      std::shared_ptr<const hdmap::LaneInfo> curr_lane);

  /**
   * @brief Check if the target lane is a right neighbor of one of some lanes.
   * @param target_lane The lane to check if it is a right neighbor.
   * @param lanes The current lanes.
   * @return If right_lane is a right neighbor of one of lanes.
   */
  static bool IsRightNeighborLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes);

  /**
   * @brief Check if the target lane is a successor of another lane.
   * @param target_lane The lane to check if it is a successor.
   * @param curr_lane The current lane.
   * @return If succ_lane is a successor of curr_lane.
   */
  static bool IsSuccessorLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      std::shared_ptr<const hdmap::LaneInfo> curr_lane);

  /**
   * @brief Check if the target lane is a successor of one of some lanes.
   * @param target_lane The lane to check if it is a successor.
   * @param lanes The current lanes.
   * @return If succ_lane is a successor of one of lanes.
   */
  static bool IsSuccessorLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes);

  /**
   * @brief Check if the target lane is a predecessor of another lane.
   * @param target_lane The lane to check if it is a predecessor.
   * @param curr_lane The current lane.
   * @return If pred_lane is a predecessor of curr_lane.
   */
  static bool IsPredecessorLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      std::shared_ptr<const hdmap::LaneInfo> curr_lane);

  /**
   * @brief Check if the target lane is a predecessor of one of some lanes.
   * @param target_lane The lane to check if it is a predecessor.
   * @param lanes The current lanes.
   * @return If pred_lane is a predecessor of one of lanes.
   */
  static bool IsPredecessorLane(
      std::shared_ptr<const hdmap::LaneInfo> target_lane,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes);

  /**
   * @brief Check if two lanes are identical.
   * @param other_lane The other lane.
   * @param curr_lane The current lane.
   * @return If other_lane is identical to curr_lane.
   */
  static bool IsIdenticalLane(std::shared_ptr<const hdmap::LaneInfo> other_lane,
                              std::shared_ptr<const hdmap::LaneInfo> curr_lane);

  /**
   * @brief Check if a lane is identical to one of some lanes.
   * @param other_lane The other lane.
   * @param lanes The lanes.
   * @return If other_lane is identical to one of lanes.
   */
  static bool IsIdenticalLane(
      std::shared_ptr<const hdmap::LaneInfo> other_lane,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes);

  /**
   * @brief Get lane turn type.
   * @param lane_id The lane ID.
   * @return Integer corresponding to the lane turn type.
   */
  static int LaneTurnType(const std::string& lane_id);

  /**
   * @brief Get all nearby lanes within certain radius given a position
   * @param position Position in ENU frame
   * @param nearyby_radius Search radius around the given position
   * @return All nearby lanes within the radius
   */
  static std::vector<std::shared_ptr<const hdmap::LaneInfo>> GetNearbyLanes(
      const common::PointENU& position, const double nearby_radius);

  /**
   * @brief Get the pointer to the lane with the smallest average curvature
   * @param The vector of lane infos
   * @return The pointer to the lane with the smallest average curvature
   */
  static std::shared_ptr<const hdmap::LaneInfo>
  LaneWithSmallestAverageCurvature(
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lane_infos);

  /**
   * @brief Get the average curvature along a lane with the ID lane_id
   * @param The ID of the lane
   * @param The size of samples alone the lane to compute the average curvature
   * @return The average curvature
   */
  static double AverageCurvature(const std::string& lane_id,
                                 const size_t sample_size);

 private:
  static std::shared_ptr<const hdmap::LaneInfo> GetNeighborLane(
      const std::shared_ptr<const hdmap::LaneInfo>& ptr_ego_lane,
      const Eigen::Vector2d& ego_position,
      const std::vector<std::string>& neighbor_lane_ids,
      const double threshold);

  PredictionMap() = delete;
};

}  // namespace prediction
}  // namespace apollo
