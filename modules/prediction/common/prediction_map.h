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
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace prediction {

class PredictionMap {
 public:
  ~PredictionMap();

  void LoadMap();

  void Clear();

  apollo::hdmap::Id id(const std::string& str_id);

  Eigen::Vector2d PositionOnLane(const apollo::hdmap::LaneInfo& lane_info,
                                 const double s);

  double HeadingOnLane(const apollo::hdmap::LaneInfo& lane_info,
                       const double s);

  double LaneTotalWidth(const apollo::hdmap::LaneInfo* lane_info_ptr,
                        const double s);

  const apollo::hdmap::LaneInfo* LaneById(const apollo::hdmap::Id& id);

  const apollo::hdmap::LaneInfo* LaneById(const std::string& str_id);

  void GetProjection(const Eigen::Vector2d& position,
                     const apollo::hdmap::LaneInfo* lane_info_ptr,
                     double* s, double* l);

  bool ProjectionFromLane(
      const apollo::hdmap::LaneInfo* lane_info_ptr,
      apollo::hdmap::PathPoint* path_point,
      double* s);

  void OnLane(
      const std::vector<const apollo::hdmap::LaneInfo*>& prev_lanes,
      const Eigen::Vector2d& point,
      const double heading,
      const double radius,
      std::vector<const apollo::hdmap::LaneInfo*>* lanes);

  int SmoothPointFromLane(
      const apollo::hdmap::Id& id,
      const double s,
      const double l,
      Eigen::Vector2d* point,
      double* heading);

  void NearbyLanesByCurrentLanes(
      const Eigen::Vector2d& point,
      const std::vector<const apollo::hdmap::LaneInfo*>& lanes,
      std::vector<const apollo::hdmap::LaneInfo*>* nearby_lanes);

  bool IsLeftNeighborLane(
      const apollo::hdmap::LaneInfo* left_lane,
      const apollo::hdmap::LaneInfo* curr_lane);

  bool IsLeftNeighborLane(
      const apollo::hdmap::LaneInfo* left_lane,
      const std::vector<const apollo::hdmap::LaneInfo*>& lanes);

  bool IsRightNeighborLane(
      const apollo::hdmap::LaneInfo* right_lane,
      const apollo::hdmap::LaneInfo* curr_lane);

  bool IsRightNeighborLane(
      const apollo::hdmap::LaneInfo* right_lane,
      const std::vector<const apollo::hdmap::LaneInfo*>& lanes);

  bool IsSuccessorLane(
      const apollo::hdmap::LaneInfo* succ_lane,
      const apollo::hdmap::LaneInfo* curr_lane);

  bool IsSuccessorLane(
      const apollo::hdmap::LaneInfo* succ_lane,
      const std::vector<const apollo::hdmap::LaneInfo*>& lanes);

  bool IsPredecessorLane(
      const apollo::hdmap::LaneInfo* pred_lane,
      const apollo::hdmap::LaneInfo* curr_lane);

  bool IsPredecessorLane(
      const apollo::hdmap::LaneInfo* pred_lane,
      const std::vector<const apollo::hdmap::LaneInfo*>& lanes);

  bool IsIdenticalLane(
      const apollo::hdmap::LaneInfo* other_lane,
      const apollo::hdmap::LaneInfo* curr_lane);

  bool IsIdenticalLane(
      const apollo::hdmap::LaneInfo* other_lane,
      const std::vector<const apollo::hdmap::LaneInfo*>& lanes);

  int LaneTurnType(const apollo::hdmap::Id& id);

  int LaneTurnType(const std::string& lane_id);

  template<class MapInfo>
  std::string id_string(const MapInfo* info) {
    return info->id().id();
  }

 private:
  std::unique_ptr<apollo::hdmap::HDMap> hdmap_;
  DECLARE_SINGLETON(PredictionMap);
};

}  // namespace prediction
}  // namespace apollo


#endif  // MODULES_PREDICTION_COMMON_PREDICTION_MAP_H_
