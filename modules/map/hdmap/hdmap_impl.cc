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

#include "modules/map/hdmap/hdmap_impl.h"

#include <algorithm>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "absl/strings/match.h"

#include "cyber/common/file.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"

namespace apollo {
namespace hdmap {
namespace {

using apollo::common::PointENU;
using apollo::common::math::AABoxKDTreeParams;
using apollo::common::math::Vec2d;

}  // namespace

HDMapImpl::HDMapImpl()
    : lanes_(signals_, overlaps_),
      stop_signs_(lanes_, junctions_),
      roads_(&lanes_, junctions_, overlaps_, parking_spaces_),
      rsus_(lanes_, junctions_, overlaps_) {}

int HDMapImpl::LoadMapFromFile(const std::string& map_filename) {
  Clear();
  // TODO(All) seems map_ can be changed to a local variable of this
  // function, but test will fail if I do so. if so.
  if (absl::EndsWith(map_filename, ".xml")) {
    if (!adapter::OpendriveAdapter::LoadData(map_filename, &map_)) {
      return -1;
    }
  } else if (!cyber::common::GetProtoFromFile(map_filename, &map_)) {
    return -1;
  }

  return LoadMapFromProto(map_);
}

int HDMapImpl::LoadMapFromProto(const Map& map_proto) {
  if (&map_proto != &map_) {  // avoid an unnecessary copy
    Clear();
    map_ = map_proto;
  }

  signals_.LoadTable(map_.signal());
  overlaps_.LoadTable(map_.overlap());
  lanes_.LoadTable(map_.lane());
  junctions_.LoadTable(map_.junction());
  crosswalks_.LoadTable(map_.crosswalk());
  stop_signs_.LoadTable(map_.stop_sign());
  yield_signs_.LoadTable(map_.yield());
  clear_areas_.LoadTable(map_.clear_area());
  speed_bumps_.LoadTable(map_.speed_bump());
  parking_spaces_.LoadTable(map_.parking_space());
  pnc_junctions_.LoadTable(map_.pnc_junction());
  roads_.LoadTable(map_.road());
  rsus_.LoadTable(map_.rsu());

  roads_.PostProcess();

  lanes_.PostProcess(*this);
  junctions_.PostProcess(*this);
  stop_signs_.PostProcess(*this);

  constexpr int default_max_leaf_size = 4;
  constexpr int lanes_max_leaf_size = 16;
  constexpr int junctions_max_leaf_size = 1;
  constexpr int crosswalks_max_leaf_size = 1;
  constexpr int pnc_junctions_max_leaf_size = 1;

  lanes_.BuildKDTree(lanes_max_leaf_size);
  junctions_.BuildKDTree(junctions_max_leaf_size);
  signals_.BuildKDTree(default_max_leaf_size);
  crosswalks_.BuildKDTree(crosswalks_max_leaf_size);
  stop_signs_.BuildKDTree(default_max_leaf_size);
  yield_signs_.BuildKDTree(default_max_leaf_size);
  clear_areas_.BuildKDTree(default_max_leaf_size);
  speed_bumps_.BuildKDTree(default_max_leaf_size);
  parking_spaces_.BuildKDTree(default_max_leaf_size);
  pnc_junctions_.BuildKDTree(pnc_junctions_max_leaf_size);
  return 0;
}

LaneInfoConstPtr HDMapImpl::GetLaneById(const Id& id) const {
  return lanes_.GetById(id);
}

JunctionInfoConstPtr HDMapImpl::GetJunctionById(const Id& id) const {
  return junctions_.GetById(id);
}

SignalInfoConstPtr HDMapImpl::GetSignalById(const Id& id) const {
  return signals_.GetById(id);
}

CrosswalkInfoConstPtr HDMapImpl::GetCrosswalkById(const Id& id) const {
  return crosswalks_.GetById(id);
}

StopSignInfoConstPtr HDMapImpl::GetStopSignById(const Id& id) const {
  return stop_signs_.GetById(id);
}

YieldSignInfoConstPtr HDMapImpl::GetYieldSignById(const Id& id) const {
  return yield_signs_.GetById(id);
}

ClearAreaInfoConstPtr HDMapImpl::GetClearAreaById(const Id& id) const {
  return clear_areas_.GetById(id);
}

SpeedBumpInfoConstPtr HDMapImpl::GetSpeedBumpById(const Id& id) const {
  return speed_bumps_.GetById(id);
}

OverlapInfoConstPtr HDMapImpl::GetOverlapById(const Id& id) const {
  return overlaps_.GetById(id);
}

RoadInfoConstPtr HDMapImpl::GetRoadById(const Id& id) const {
  return roads_.GetById(id);
}

ParkingSpaceInfoConstPtr HDMapImpl::GetParkingSpaceById(const Id& id) const {
  return parking_spaces_.GetById(id);
}

PNCJunctionInfoConstPtr HDMapImpl::GetPNCJunctionById(const Id& id) const {
  return pnc_junctions_.GetById(id);
}

RSUInfoConstPtr HDMapImpl::GetRSUById(const Id& id) const {
  return rsus_.GetById(id);
}

int HDMapImpl::GetLanes(const PointENU& point, double distance,
                        std::vector<LaneInfoConstPtr>* lanes) const {
  return lanes_.GetItems(point, distance, lanes);
}

int HDMapImpl::GetLanes(const Vec2d& point, double distance,
                        std::vector<LaneInfoConstPtr>* lanes) const {
  return lanes_.GetItems(point, distance, lanes);
}

int HDMapImpl::GetRoads(const PointENU& point, double distance,
                        std::vector<RoadInfoConstPtr>* roads) const {
  return GetRoads({point.x(), point.y()}, distance, roads);
}

int HDMapImpl::GetRoads(const Vec2d& point, double distance,
                        std::vector<RoadInfoConstPtr>* roads) const {
  return roads_.GetItems(point, distance, roads);
}

int HDMapImpl::GetJunctions(
    const PointENU& point, double distance,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  return junctions_.GetItems(point, distance, junctions);
}

int HDMapImpl::GetJunctions(
    const Vec2d& point, double distance,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  return junctions_.GetItems(point, distance, junctions);
}

int HDMapImpl::GetSignals(const PointENU& point, double distance,
                          std::vector<SignalInfoConstPtr>* signals) const {
  return signals_.GetItems(point, distance, signals);
}

int HDMapImpl::GetSignals(const Vec2d& point, double distance,
                          std::vector<SignalInfoConstPtr>* signals) const {
  return signals_.GetItems(point, distance, signals);
}

int HDMapImpl::GetCrosswalks(
    const PointENU& point, double distance,
    std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  return crosswalks_.GetItems(point, distance, crosswalks);
}

int HDMapImpl::GetCrosswalks(
    const Vec2d& point, double distance,
    std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  return crosswalks_.GetItems(point, distance, crosswalks);
}

int HDMapImpl::GetStopSigns(
    const PointENU& point, double distance,
    std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return stop_signs_.GetItems(point, distance, stop_signs);
}

int HDMapImpl::GetStopSigns(
    const Vec2d& point, double distance,
    std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return stop_signs_.GetItems(point, distance, stop_signs);
}

int HDMapImpl::GetYieldSigns(
    const PointENU& point, double distance,
    std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  return yield_signs_.GetItems(point, distance, yield_signs);
}

int HDMapImpl::GetYieldSigns(
    const Vec2d& point, double distance,
    std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  return yield_signs_.GetItems(point, distance, yield_signs);
}

int HDMapImpl::GetClearAreas(
    const PointENU& point, double distance,
    std::vector<ClearAreaInfoConstPtr>* clear_areas) const {
  return clear_areas_.GetItems(point, distance, clear_areas);
}

int HDMapImpl::GetClearAreas(
    const Vec2d& point, double distance,
    std::vector<ClearAreaInfoConstPtr>* clear_areas) const {
  return clear_areas_.GetItems(point, distance, clear_areas);
}

int HDMapImpl::GetSpeedBumps(
    const PointENU& point, double distance,
    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const {
  return speed_bumps_.GetItems(point, distance, speed_bumps);
}

int HDMapImpl::GetSpeedBumps(
    const Vec2d& point, double distance,
    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const {
  return speed_bumps_.GetItems(point, distance, speed_bumps);
}

int HDMapImpl::GetParkingSpaces(
    const PointENU& point, double distance,
    std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const {
  return parking_spaces_.GetItems(point, distance, parking_spaces);
}

int HDMapImpl::GetParkingSpaces(
    const Vec2d& point, double distance,
    std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const {
  return parking_spaces_.GetItems(point, distance, parking_spaces);
}

int HDMapImpl::GetPNCJunctions(
    const apollo::common::PointENU& point, double distance,
    std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const {
  return pnc_junctions_.GetItems(point, distance, pnc_junctions);
}

int HDMapImpl::GetPNCJunctions(
    const apollo::common::math::Vec2d& point, double distance,
    std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const {
  return pnc_junctions_.GetItems(point, distance, pnc_junctions);
}

int HDMapImpl::GetNearestLane(const PointENU& point,
                              LaneInfoConstPtr* nearest_lane, double* nearest_s,
                              double* nearest_l) const {
  return GetNearestLane({point.x(), point.y()}, nearest_lane, nearest_s,
                        nearest_l);
}

int HDMapImpl::GetNearestLane(const Vec2d& point,
                              LaneInfoConstPtr* nearest_lane, double* nearest_s,
                              double* nearest_l) const {
  return lanes_.GetNearest(point, nearest_lane, nearest_s, nearest_l);
}

int HDMapImpl::GetNearestLaneWithHeading(
    const PointENU& point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr* nearest_lane,
    double* nearest_s, double* nearest_l) const {
  return GetNearestLaneWithHeading({point.x(), point.y()}, distance,
                                   central_heading, max_heading_difference,
                                   nearest_lane, nearest_s, nearest_l);
}

int HDMapImpl::GetNearestLaneWithHeading(
    const Vec2d& point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr* nearest_lane,
    double* nearest_s, double* nearest_l) const {
  return lanes_.GetNearestWithHeading(point, distance, central_heading,
                                      max_heading_difference, nearest_lane,
                                      nearest_s, nearest_l);
}

int HDMapImpl::GetLanesWithHeading(const PointENU& point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  return lanes_.GetItemsWithHeading(point, distance, central_heading,
                                    max_heading_difference, lanes);
}

int HDMapImpl::GetLanesWithHeading(const Vec2d& point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  return lanes_.GetItemsWithHeading(point, distance, central_heading,
                                    max_heading_difference, lanes);
}

int HDMapImpl::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadROIBoundaryPtr>* road_boundaries,
    std::vector<JunctionBoundaryPtr>* junctions) const {
  return roads_.GetRoadBoundaries(point, radius, road_boundaries, junctions);
}

int HDMapImpl::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  return roads_.GetRoadBoundaries(point, radius, road_boundaries, junctions);
}

int HDMapImpl::GetRoi(const apollo::common::PointENU& point, double radius,
                      std::vector<RoadRoiPtr>* roads_roi,
                      std::vector<PolygonRoiPtr>* polygons_roi) {
  return roads_.GetRoi(point, radius, roads_roi, polygons_roi);
}

int HDMapImpl::GetForwardNearestSignalsOnLane(
    const apollo::common::PointENU& point, const double distance,
    std::vector<SignalInfoConstPtr>* signals) const {
  return lanes_.GetForwardNearestSignalsOnLane(point, distance, signals);
}

int HDMapImpl::GetStopSignAssociatedStopSigns(
    const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return stop_signs_.GetAssociatedSigns(id, stop_signs);
}

int HDMapImpl::GetStopSignAssociatedLanes(
    const Id& id, std::vector<LaneInfoConstPtr>* lanes) const {
  return stop_signs_.GetAssociatedLanes(id, lanes);
}

template <typename MapAccessFunction, typename Objects>
void AddItems(std::vector<Id>* dest_overlap_ids,
              std::unordered_set<std::string>* dest_map_element_ids,
              const Objects& src_objects,
              MapAccessFunction map_access_function) {
  for (auto& item_ptr : src_objects) {
    dest_map_element_ids->insert(item_ptr->id().id());
    auto& src_object = item_ptr->inner_object();
    auto& src_overlap_ids = src_object.overlap_id();
    std::copy(src_overlap_ids.begin(), src_overlap_ids.end(),
              std::back_inserter(*dest_overlap_ids));
    *map_access_function() = src_object;
  }
}

void AddOverlaps(Map* local_map, const std::vector<Id>& overlap_ids,
                 const std::unordered_set<std::string>& map_element_ids,
                 const objects::Overlaps& overlaps) {
  for (auto& overlap_id : overlap_ids) {
    auto overlap_ptr = overlaps.GetById(overlap_id);
    if (overlap_ptr == nullptr) {
      AERROR << "overlpa id [" << overlap_id.id() << "] is not found.";
      continue;
    }

    bool need_delete = false;
    for (auto& overlap_object : overlap_ptr->inner_object().object()) {
      if (map_element_ids.count(overlap_object.id().id()) <= 0) {
        need_delete = true;
        break;
      }
    }

    if (!need_delete) {
      *local_map->add_overlap() = overlap_ptr->inner_object();
    }
  }
}

void AddRoads(Map* local_map, std::unordered_set<std::string>* map_element_ids,
              const std::vector<RoadInfoConstPtr>& roads) {
  for (auto& road_ptr : roads) {
    map_element_ids->insert(road_ptr->id().id());
    *local_map->add_road() = road_ptr->inner_object();
  }
}

int HDMapImpl::GetLocalMap(const apollo::common::PointENU& point,
                           const std::pair<double, double>& range,
                           Map* local_map) const {
  CHECK_NOTNULL(local_map);

  double distance = std::max(range.first, range.second);
  CHECK_GT(distance, 0.0);

  std::vector<LaneInfoConstPtr> lanes;
  lanes_.GetItems(point, distance, &lanes);

  std::vector<JunctionInfoConstPtr> junctions;
  junctions_.GetItems(point, distance, &junctions);

  std::vector<CrosswalkInfoConstPtr> crosswalks;
  crosswalks_.GetItems(point, distance, &crosswalks);

  std::vector<SignalInfoConstPtr> signals;
  signals_.GetItems(point, distance, &signals);

  std::vector<StopSignInfoConstPtr> stop_signs;
  stop_signs_.GetItems(point, distance, &stop_signs);

  std::vector<YieldSignInfoConstPtr> yield_signs;
  yield_signs_.GetItems(point, distance, &yield_signs);

  std::vector<ClearAreaInfoConstPtr> clear_areas;
  clear_areas_.GetItems(point, distance, &clear_areas);

  std::vector<SpeedBumpInfoConstPtr> speed_bumps;
  speed_bumps_.GetItems(point, distance, &speed_bumps);

  std::vector<RoadInfoConstPtr> roads;
  roads_.GetItems(point, distance, &roads);

  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  parking_spaces_.GetItems(point, distance, &parking_spaces);

  const size_t map_element_ids_capacity =
      lanes.size() + crosswalks.size() + junctions.size() + signals.size() +
      stop_signs.size() + yield_signs.size() + clear_areas.size() +
      speed_bumps.size() + parking_spaces.size() + roads.size();

  std::unordered_set<std::string> map_element_ids;
  map_element_ids.reserve(map_element_ids_capacity);

  std::vector<Id> overlap_ids;

  AddItems(&overlap_ids, &map_element_ids, lanes,
           [=]() { return local_map->add_lane(); });

  AddItems(&overlap_ids, &map_element_ids, crosswalks,
           [=]() { return local_map->add_crosswalk(); });

  AddItems(&overlap_ids, &map_element_ids, junctions,
           [=]() { return local_map->add_junction(); });

  AddItems(&overlap_ids, &map_element_ids, signals,
           [=]() { return local_map->add_signal(); });

  AddItems(&overlap_ids, &map_element_ids, stop_signs,
           [=]() { return local_map->add_stop_sign(); });

  AddItems(&overlap_ids, &map_element_ids, yield_signs,
           [=]() { return local_map->add_yield(); });

  AddItems(&overlap_ids, &map_element_ids, clear_areas,
           [=]() { return local_map->add_clear_area(); });

  AddItems(&overlap_ids, &map_element_ids, speed_bumps,
           [=]() { return local_map->add_speed_bump(); });

  AddItems(&overlap_ids, &map_element_ids, parking_spaces,
           [=]() { return local_map->add_parking_space(); });

  AddRoads(local_map, &map_element_ids, roads);

  AddOverlaps(local_map, overlap_ids, map_element_ids, overlaps_);

  return 0;
}

int HDMapImpl::GetForwardNearestRSUs(const apollo::common::PointENU& point,
                                     double distance, double central_heading,
                                     double max_heading_difference,
                                     std::vector<RSUInfoConstPtr>* rsus) const {
  return rsus_.GetForwardNearestItems(point, distance, central_heading,
                                      max_heading_difference, rsus);
}

void HDMapImpl::Clear() {
  map_.Clear();
  lanes_.Clear();
  junctions_.Clear();
  crosswalks_.Clear();
  signals_.Clear();
  stop_signs_.Clear();
  yield_signs_.Clear();
  clear_areas_.Clear();
  speed_bumps_.Clear();
  parking_spaces_.Clear();
  pnc_junctions_.Clear();
  overlaps_.Clear();
  roads_.Clear();
  rsus_.Clear();
}

}  // namespace hdmap
}  // namespace apollo
