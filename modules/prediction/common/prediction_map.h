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

#ifndef MODULES_PREDICTION_COMMON_PREDICTION_MAP_H_
#define MODULES_PREDICTION_COMMON_PREDICTION_MAP_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/macro.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace prediction {

class PredictionMap {
 public:
  apollo::hdmap::Id id(const std::string& str_id);

  static Eigen::Vector2d PositionOnLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info,
      const double s);

  static double HeadingOnLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info,
      const double s);

  static double LaneTotalWidth(
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr,
      const double s);

  std::shared_ptr<const apollo::hdmap::LaneInfo> LaneById(
      const apollo::hdmap::Id& id);

  std::shared_ptr<const apollo::hdmap::LaneInfo> LaneById(
      const std::string& str_id);

  static void GetProjection(
      const Eigen::Vector2d& position,
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr,
      double* s,
      double* l);

  static bool ProjectionFromLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr,
      double s, apollo::hdmap::MapPathPoint* path_point);

  void OnLane(
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>&
          prev_lanes,
      const Eigen::Vector2d& point, const double heading,
      const double radius,
      std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>* lanes,
      bool on_lane = true);

  static double PathHeading(
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr,
      const apollo::common::PointENU& point);

  int SmoothPointFromLane(
      const apollo::hdmap::Id& id, const double s,
      const double l, Eigen::Vector2d* point,
      double* heading);

  void NearbyLanesByCurrentLanes(
      const Eigen::Vector2d& point,
      double heading,
      double radius,
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>& lanes,
      std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>*
          nearby_lanes);

  static bool IsLeftNeighborLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> left_lane,
      std::shared_ptr<const apollo::hdmap::LaneInfo> curr_lane);

  static bool IsLeftNeighborLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> left_lane,
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>& lanes);

  static bool IsRightNeighborLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> right_lane,
      std::shared_ptr<const apollo::hdmap::LaneInfo> curr_lane);

  static bool IsRightNeighborLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> right_lane,
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>& lanes);

  static bool IsSuccessorLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> succ_lane,
      std::shared_ptr<const apollo::hdmap::LaneInfo> curr_lane);

  static bool IsSuccessorLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> succ_lane,
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>& lanes);

  static bool IsPredecessorLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> pred_lane,
      std::shared_ptr<const apollo::hdmap::LaneInfo> curr_lane);

  static bool IsPredecessorLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> pred_lane,
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>& lanes);

  static bool IsIdenticalLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> other_lane,
      std::shared_ptr<const apollo::hdmap::LaneInfo> curr_lane);

  static bool IsIdenticalLane(
      std::shared_ptr<const apollo::hdmap::LaneInfo> other_lane,
      const std::vector<std::shared_ptr<const apollo::hdmap::LaneInfo>>& lanes);

  int LaneTurnType(const apollo::hdmap::Id& id);

  int LaneTurnType(const std::string& lane_id);

  template <class MapInfo>
  static std::string id_string(std::shared_ptr<const MapInfo> info) {
    return info->id().id();
  }

 private:
  bool LoadMap();

  std::unique_ptr<apollo::hdmap::HDMap> hdmap_ = nullptr;
  DECLARE_SINGLETON(PredictionMap);
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_PREDICTION_MAP_H_
