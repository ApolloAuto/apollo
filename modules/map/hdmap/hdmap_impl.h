/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#ifndef MODULES_MAP_HDMAP_HDMAP_IMPL_H
#define MODULES_MAP_HDMAP_HDMAP_IMPL_H

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/aaboxkdtree2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/proto/map.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/hdmap/hdmap_common.h"

/**
 * @namespace apollo::hdmap
 * @brief apollo::hdmap
 */
namespace apollo {
namespace hdmap {

/**
 * @class HDMapImpl
 *
 * @brief High-precision map loader implement.
 */
class HDMapImpl {
 public:
  using LaneTable =
      std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
  using JunctionTable =
      std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
  using SignalTable =
      std::unordered_map<std::string, std::shared_ptr<SignalInfo>>;
  using CrosswalkTable =
      std::unordered_map<std::string, std::shared_ptr<CrosswalkInfo>>;
  using StopSignTable =
      std::unordered_map<std::string, std::shared_ptr<StopSignInfo>>;
  using YieldSignTable =
      std::unordered_map<std::string, std::shared_ptr<YieldSignInfo>>;
  using OverlapTable =
      std::unordered_map<std::string, std::shared_ptr<OverlapInfo>>;
  using RoadTable =
      std::unordered_map<std::string, std::shared_ptr<RoadInfo>>;
 public:
  /**
  * @brief load map from local file
  * @param map_filename path of map data file
  * @return 0:success, otherwise failed
  */
  int LoadMapFromFile(const std::string& map_filename);

  LaneInfoConstPtr GetLaneById(const Id& id) const;
  JunctionInfoConstPtr GetJunctionById(const Id& id) const;
  SignalInfoConstPtr GetSignalById(const Id& id) const;
  CrosswalkInfoConstPtr GetCrosswalkById(const Id& id) const;
  StopSignInfoConstPtr GetStopSignById(const Id& id) const;
  YieldSignInfoConstPtr GetYieldSignById(const Id& id) const;
  OverlapInfoConstPtr GetOverlapById(const Id& id) const;
  RoadInfoConstPtr GetRoadById(const Id& id) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const apollo::common::PointENU& point,
               double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetJunctions(const apollo::common::PointENU& point,
                   double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get all crosswalks in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param crosswalks store all crosswalks in target range
   * @return 0:success, otherwise failed
   */
  int GetCrosswalks(const apollo::common::PointENU& point,
                    double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const apollo::common::PointENU& point,
                 double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  /**
   * @brief get all stop signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param stop signs store all stop signs in target range
   * @return 0:success, otherwise failed
   */
  int GetStopSigns(const apollo::common::PointENU& point,
                   double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  /**
   * @brief get all yield signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param yield signs store all yield signs in target range
   * @return 0:success, otherwise failed
   */
  int GetYieldSigns(const apollo::common::PointENU& point,
                    double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  /**
   * @brief get all roads in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param roads store all roads in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const apollo::common::PointENU& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const apollo::common::PointENU& point,
                     LaneInfoConstPtr* nearest_lane,
                     double* nearest_s,
                     double* nearest_l) const;
  /**
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const apollo::common::PointENU& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s,
                                double* nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const apollo::common::PointENU& point,
                          const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const apollo::common::PointENU& point,
                        double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;

 private:
  int GetLanes(const apollo::common::math::Vec2d& point,
               double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  int GetJunctions(const apollo::common::math::Vec2d& point,
                   double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  int GetCrosswalks(const apollo::common::math::Vec2d& point,
                    double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  int GetSignals(const apollo::common::math::Vec2d& point,
                 double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  int GetStopSigns(const apollo::common::math::Vec2d& point,
                   double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  int GetYieldSigns(const apollo::common::math::Vec2d& point,
                    double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  int GetNearestLane(const apollo::common::math::Vec2d &point,
                     LaneInfoConstPtr* nearest_lane,
                     double *nearest_s,
                     double *nearest_l) const;
  int GetNearestLaneWithHeading(const apollo::common::math::Vec2d& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s,
                                double* nearest_l) const;
  int GetLanesWithHeading(const apollo::common::math::Vec2d &point,
                          const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr> *lanes) const;
  int GetRoads(const apollo::common::math::Vec2d& point,
               double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  template<class Table, class BoxTable, class KDTree>
  static void BuildSegmentKDTree(
      const Table& table,
      const apollo::common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table,
      std::unique_ptr<KDTree>* const kdtree);

  template<class Table, class BoxTable, class KDTree>
  static void BuildPolygonKDTree(
      const Table& table,
      const apollo::common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table,
      std::unique_ptr<KDTree>* const kdtree);

  void BuildLaneSegmentKDTree();
  void BuildJunctionPolygonKDTree();
  void BuildCrosswalkPolygonKDTree();
  void BuildSignalSegmentKDTree();
  void BuildStopSignSegmentKDTree();
  void BuildYieldSignSegmentKDTree();

  template<class KDTree>
  static int SearchObjects(const apollo::common::math::Vec2d& center,
                           const double radius,
                           const KDTree& kdtree,
                           std::vector<std::string>* const results);

  void Clear();

 private:
  Map _map;

  LaneTable           _lane_table;
  JunctionTable       _junction_table;
  CrosswalkTable      _crosswalk_table;
  SignalTable         _signal_table;
  StopSignTable       _stop_sign_table;
  YieldSignTable      _yield_sign_table;
  OverlapTable        _overlap_table;
  RoadTable           _road_table;

  std::vector<LaneSegmentBox> _lane_segment_boxes;
  std::unique_ptr<LaneSegmentKDTree> _lane_segment_kdtree;

  std::vector<JunctionPolygonBox> _junction_polygon_boxes;
  std::unique_ptr<JunctionPolygonKDTree> _junction_polygon_kdtree;

  std::vector<CrosswalkPolygonBox> _crosswalk_polygon_boxes;
  std::unique_ptr<CrosswalkPolygonKDTree> _crosswalk_polygon_kdtree;

  std::vector<SignalSegmentBox> _signal_segment_boxes;
  std::unique_ptr<SignalSegmentKDTree> _signal_segment_kdtree;

  std::vector<StopSignSegmentBox> _stop_sign_segment_boxes;
  std::unique_ptr<StopSignSegmentKDTree> _stop_sign_segment_kdtree;

  std::vector<YieldSignSegmentBox> _yield_sign_segment_boxes;
  std::unique_ptr<YieldSignSegmentKDTree> _yield_sign_segment_kdtree;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_HDMAP_IMPL_H
