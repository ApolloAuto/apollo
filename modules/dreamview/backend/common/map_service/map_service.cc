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

#include "modules/dreamview/backend/common/map_service/map_service.h"

#include <algorithm>
#include <fstream>
#include <utility>

#include "modules/common/util/json_util.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::PointENU;
using apollo::hdmap::AreaInfoConstPtr;
using apollo::hdmap::BarrierGateInfoConstPtr;
using apollo::hdmap::ClearAreaInfoConstPtr;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::Id;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::Lane;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::Map;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PNCJunctionInfoConstPtr;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::hdmap::SpeedBumpInfoConstPtr;
using apollo::hdmap::StopSignInfoConstPtr;
using apollo::hdmap::YieldSignInfoConstPtr;
using apollo::routing::RoutingResponse;
using google::protobuf::RepeatedPtrField;

namespace {

template <typename MapElementInfoConstPtr>
void ExtractIds(const std::vector<MapElementInfoConstPtr> &items,
                RepeatedPtrField<std::string> *ids) {
  ids->Reserve(static_cast<unsigned int>(items.size()));
  for (const auto &item : items) {
    ids->Add()->assign(item->id().id());
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractRoadAndLaneIds(const std::vector<LaneInfoConstPtr> &lanes,
                           RepeatedPtrField<std::string> *lane_ids,
                           RepeatedPtrField<std::string> *road_ids) {
  lane_ids->Reserve(static_cast<unsigned int>(lanes.size()));
  road_ids->Reserve(static_cast<unsigned int>(lanes.size()));

  for (const auto &lane : lanes) {
    lane_ids->Add()->assign(lane->id().id());
    if (!lane->road_id().id().empty()) {
      road_ids->Add()->assign(lane->road_id().id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(lane_ids->begin(), lane_ids->end());
  std::sort(road_ids->begin(), road_ids->end());
}

}  // namespace

const char MapService::kMetaFileName[] = "/metaInfo.json";

MapService::MapService(bool use_sim_map) : use_sim_map_(use_sim_map) {
  ReloadMap(false);
}

bool MapService::ReloadMap(bool force_reload) {
  boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
  bool ret = true;
  if (force_reload) {
    ret = HDMapUtil::ReloadMaps();
  }

  // Update the x,y-offsets if present.
  // UpdateOffsets();
  return ret;
}

void MapService::UpdateOffsets() {
  x_offset_ = 0.0;
  y_offset_ = 0.0;
  std::ifstream ifs(FLAGS_map_dir + kMetaFileName);
  if (!ifs.is_open()) {
    AINFO << "Failed to open map meta file: " << kMetaFileName;
  } else {
    nlohmann::json json;
    ifs >> json;
    ifs.close();

    for (auto it = json.begin(); it != json.end(); ++it) {
      auto val = it.value();
      if (val.is_object()) {
        auto x_offset = val.find("xoffset");
        if (x_offset == val.end()) {
          AWARN << "Cannot find x_offset for this map " << it.key();
          continue;
        }

        if (!x_offset->is_number()) {
          AWARN << "Expect x_offset with type 'number', but was "
                << x_offset->type_name();
          continue;
        }
        x_offset_ = x_offset.value();

        auto y_offset = val.find("yoffset");
        if (y_offset == val.end()) {
          AWARN << "Cannot find y_offset for this map " << it.key();
          continue;
        }

        if (!y_offset->is_number()) {
          AWARN << "Expect y_offset with type 'number', but was "
                << y_offset->type_name();
          continue;
        }
        y_offset_ = y_offset.value();
      }
    }
  }
  AINFO << "Updated with map: x_offset " << x_offset_ << ", y_offset "
        << y_offset_;
}

const hdmap::HDMap *MapService::HDMap() const {
  return HDMapUtil::BaseMapPtr();
}

const hdmap::HDMap *MapService::SimMap() const {
  return use_sim_map_ ? HDMapUtil::SimMapPtr() : HDMapUtil::BaseMapPtr();
}

bool MapService::MapReady() const { return HDMap() && SimMap(); }

bool MapService::PointIsValid(const double x, const double y) const {
  MapElementIds ids;
  PointENU point;
  point.set_x(x);
  point.set_y(y);
  CollectMapElementIds(point, 20, &ids);
  return (ids.ByteSizeLong() != 0);
}

void MapService::CollectMapElementIds(const PointENU &point, double radius,
                                      MapElementIds *ids) const {
  if (!MapReady()) {
    return;
  }
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  std::vector<LaneInfoConstPtr> lanes;
  if (SimMap()->GetLanes(point, radius, &lanes) != 0) {
    AERROR << "Fail to get lanes from sim_map.";
  }
  ExtractRoadAndLaneIds(lanes, ids->mutable_lane(), ids->mutable_road());

  std::vector<ClearAreaInfoConstPtr> clear_areas;
  if (SimMap()->GetClearAreas(point, radius, &clear_areas) != 0) {
    AERROR << "Fail to get clear areas from sim_map.";
  }
  ExtractIds(clear_areas, ids->mutable_clear_area());

  std::vector<CrosswalkInfoConstPtr> crosswalks;
  if (SimMap()->GetCrosswalks(point, radius, &crosswalks) != 0) {
    AERROR << "Fail to get crosswalks from sim_map.";
  }
  ExtractIds(crosswalks, ids->mutable_crosswalk());

  std::vector<JunctionInfoConstPtr> junctions;
  if (SimMap()->GetJunctions(point, radius, &junctions) != 0) {
    AERROR << "Fail to get junctions from sim_map.";
  }
  ExtractIds(junctions, ids->mutable_junction());

  std::vector<PNCJunctionInfoConstPtr> pnc_junctions;
  if (SimMap()->GetPNCJunctions(point, radius, &pnc_junctions) != 0) {
    AERROR << "Fail to get pnc junctions from sim_map.";
  }
  ExtractIds(pnc_junctions, ids->mutable_pnc_junction());

  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (SimMap()->GetParkingSpaces(point, radius, &parking_spaces) != 0) {
    AERROR << "Fail to get parking space from sim_map.";
  }
  ExtractIds(parking_spaces, ids->mutable_parking_space());

  std::vector<SpeedBumpInfoConstPtr> speed_bumps;
  if (SimMap()->GetSpeedBumps(point, radius, &speed_bumps) != 0) {
    AERROR << "Fail to get speed bump from sim_map.";
  }
  ExtractIds(speed_bumps, ids->mutable_speed_bump());

  std::vector<SignalInfoConstPtr> signals;
  if (SimMap()->GetSignals(point, radius, &signals) != 0) {
    AERROR << "Failed to get signals from sim_map.";
  }

  ExtractIds(signals, ids->mutable_signal());

  std::vector<StopSignInfoConstPtr> stop_signs;
  if (SimMap()->GetStopSigns(point, radius, &stop_signs) != 0) {
    AERROR << "Failed to get stop signs from sim_map.";
  }
  ExtractIds(stop_signs, ids->mutable_stop_sign());

  std::vector<YieldSignInfoConstPtr> yield_signs;
  if (SimMap()->GetYieldSigns(point, radius, &yield_signs) != 0) {
    AERROR << "Failed to get yield signs from sim_map.";
  }
  ExtractIds(yield_signs, ids->mutable_yield());

  std::vector<AreaInfoConstPtr> ad_areas;
  if (SimMap()->GetAreas(point, radius, &ad_areas) != 0) {
    AERROR << "Failed to get ad areas from sim_map.";
  }
  ExtractIds(ad_areas, ids->mutable_ad_area());

  std::vector<BarrierGateInfoConstPtr> barrier_gates;
  if (SimMap()->GetBarrierGates(point, radius, &barrier_gates) != 0) {
    AERROR << "Failed to get barrier gate from sim_map.";
  }
  ExtractIds(barrier_gates, ids->mutable_barrier_gate());
}

Map MapService::RetrieveMapElements(const MapElementIds &ids) const {
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  Map result;
  if (!MapReady()) {
    return result;
  }
  Id map_id;

  for (const auto &id : ids.lane()) {
    map_id.set_id(id);
    auto element = SimMap()->GetLaneById(map_id);
    if (element) {
      auto lane = element->lane();
      lane.clear_left_sample();
      lane.clear_right_sample();
      lane.clear_left_road_sample();
      lane.clear_right_road_sample();
      *result.add_lane() = lane;
    }
  }

  for (const auto &id : ids.clear_area()) {
    map_id.set_id(id);
    auto element = SimMap()->GetClearAreaById(map_id);
    if (element) {
      *result.add_clear_area() = element->clear_area();
    }
  }

  for (const auto &id : ids.crosswalk()) {
    map_id.set_id(id);
    auto element = SimMap()->GetCrosswalkById(map_id);
    if (element) {
      *result.add_crosswalk() = element->crosswalk();
    }
  }

  for (const auto &id : ids.junction()) {
    map_id.set_id(id);
    auto element = SimMap()->GetJunctionById(map_id);
    if (element) {
      *result.add_junction() = element->junction();
    }
  }

  for (const auto &id : ids.signal()) {
    map_id.set_id(id);
    auto element = SimMap()->GetSignalById(map_id);
    if (element) {
      *result.add_signal() = element->signal();
    }
  }

  for (const auto &id : ids.stop_sign()) {
    map_id.set_id(id);
    auto element = SimMap()->GetStopSignById(map_id);
    if (element) {
      *result.add_stop_sign() = element->stop_sign();
    }
  }

  for (const auto &id : ids.yield()) {
    map_id.set_id(id);
    auto element = SimMap()->GetYieldSignById(map_id);
    if (element) {
      *result.add_yield() = element->yield_sign();
    }
  }

  for (const auto &id : ids.road()) {
    map_id.set_id(id);
    auto element = SimMap()->GetRoadById(map_id);
    if (element) {
      *result.add_road() = element->road();
    }
  }

  for (const auto &id : ids.parking_space()) {
    map_id.set_id(id);
    auto element = SimMap()->GetParkingSpaceById(map_id);
    if (element) {
      *result.add_parking_space() = element->parking_space();
    }
  }

  for (const auto &id : ids.speed_bump()) {
    map_id.set_id(id);
    auto element = SimMap()->GetSpeedBumpById(map_id);
    if (element) {
      *result.add_speed_bump() = element->speed_bump();
    }
  }

  for (const auto &id : ids.pnc_junction()) {
    map_id.set_id(id);
    auto element = SimMap()->GetPNCJunctionById(map_id);
    if (element) {
      *result.add_pnc_junction() = element->pnc_junction();
    }
  }

  for (const auto &id : ids.ad_area()) {
    map_id.set_id(id);
    auto element = SimMap()->GetAreaById(map_id);
    if (element) {
      *result.add_ad_area() = element->area();
    }
  }

  for (const auto &id : ids.barrier_gate()) {
    map_id.set_id(id);
    auto element = SimMap()->GetBarrierGateById(map_id);
    if (element) {
      *result.add_barrier_gate() = element->barrier_gate();
    }
  }

  apollo::hdmap::Header map_header;
  if (SimMap()->GetMapHeader(&map_header)) {
    result.mutable_header()->CopyFrom(map_header);
  }

  return result;
}

bool MapService::GetNearestLaneWithDistance(
    const double x, const double y,
    apollo::hdmap::LaneInfoConstPtr *nearest_lane, double *nearest_s,
    double *nearest_l) const {
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  PointENU point;
  point.set_x(x);
  point.set_y(y);
  static constexpr double kSearchRadius = 3.0;
  if (!MapReady() ||
      HDMap()->GetNearestLaneWithDistance(point, kSearchRadius, nearest_lane,
                                          nearest_s, nearest_l) < 0) {
    AERROR << "Failed to get nearest lane!";
    return false;
  }
  return true;
}

bool MapService::GetNearestLane(const double x, const double y,
                                LaneInfoConstPtr *nearest_lane,
                                double *nearest_s, double *nearest_l) const {
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  PointENU point;
  point.set_x(x);
  point.set_y(y);
  if (!MapReady() ||
      HDMap()->GetNearestLane(point, nearest_lane, nearest_s, nearest_l) < 0) {
    AERROR << "Failed to get nearest lane!";
    return false;
  }
  return true;
}

bool MapService::GetNearestLaneWithHeading(const double x, const double y,
                                           LaneInfoConstPtr *nearest_lane,
                                           double *nearest_s, double *nearest_l,
                                           const double heading) const {
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  PointENU point;
  point.set_x(x);
  point.set_y(y);
  static constexpr double kSearchRadius = 3.0;
  static constexpr double kMaxHeadingDiff = 1.0;
  if (!MapReady() || HDMap()->GetNearestLaneWithHeading(
                         point, kSearchRadius, heading, kMaxHeadingDiff,
                         nearest_lane, nearest_s, nearest_l) < 0) {
    AERROR << "Failed to get nearest lane with heading.";
    return false;
  }
  return true;
}

bool MapService::GetPathsFromRouting(const RoutingResponse &routing,
                                     std::vector<Path> *paths) const {
  if (!CreatePathsFromRouting(routing, paths)) {
    AERROR << "Unable to get paths from routing!";
    return false;
  }
  return true;
}

bool MapService::GetPoseWithRegardToLane(const double x, const double y,
                                         double *theta, double *s) const {
  double l;
  LaneInfoConstPtr nearest_lane;
  if (!GetNearestLane(x, y, &nearest_lane, s, &l)) {
    return false;
  }

  *theta = nearest_lane->Heading(*s);
  return true;
}

bool MapService::ConstructLaneWayPoint(
    const double x, const double y, routing::LaneWaypoint *laneWayPoint) const {
  double s, l;
  LaneInfoConstPtr lane;
  if (!GetNearestLane(x, y, &lane, &s, &l)) {
    return false;
  }

  if (!CheckRoutingPointLaneType(lane)) {
    return false;
  }

  laneWayPoint->set_id(lane->id().id());
  laneWayPoint->set_s(s);
  auto *pose = laneWayPoint->mutable_pose();
  pose->set_x(x);
  pose->set_y(y);

  return true;
}

bool MapService::ConstructLaneWayPointWithHeading(
    const double x, const double y, const double heading,
    routing::LaneWaypoint *laneWayPoint) const {
  double s, l;
  LaneInfoConstPtr lane;
  if (!GetNearestLaneWithHeading(x, y, &lane, &s, &l, heading)) {
    return false;
  }

  if (!CheckRoutingPointLaneType(lane)) {
    return false;
  }

  laneWayPoint->set_id(lane->id().id());
  laneWayPoint->set_s(s);
  auto *pose = laneWayPoint->mutable_pose();
  pose->set_x(x);
  pose->set_y(y);

  return true;
}

bool MapService::ConstructLaneWayPointWithLaneId(
    const double x, const double y, const std::string id,
    routing::LaneWaypoint *laneWayPoint) const {
  LaneInfoConstPtr lane = HDMap()->GetLaneById(hdmap::MakeMapId(id));
  if (!lane) {
    return false;
  }

  if (!CheckRoutingPointLaneType(lane)) {
    return false;
  }

  double s, l;
  PointENU point;
  point.set_x(x);
  point.set_y(y);

  if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
    return false;
  }

  // Limit s with max value of the length of the lane, or not the laneWayPoint
  // may be invalid.
  if (s > lane->lane().length()) {
    s = lane->lane().length();
  }

  laneWayPoint->set_id(id);
  laneWayPoint->set_s(s);
  auto *pose = laneWayPoint->mutable_pose();
  pose->set_x(x);
  pose->set_y(y);

  return true;
}

bool MapService::CheckRoutingPoint(const double x, const double y) const {
  double s, l;
  LaneInfoConstPtr lane;
  if (!GetNearestLaneWithDistance(x, y, &lane, &s, &l)) {
    if (GetParkingSpaceId(x, y) != "-1") {
      return true;
    }
    return false;
  }
  if (!CheckRoutingPointLaneType(lane)) {
    return false;
  }
  return true;
}

bool MapService::CheckRoutingPointWithHeading(const double x, const double y,
                                              const double heading) const {
  double s, l;
  LaneInfoConstPtr lane;
  if (!GetNearestLaneWithHeading(x, y, &lane, &s, &l, heading)) {
    if (GetParkingSpaceId(x, y) != "-1") {
      return true;
    }
    return false;
  }
  if (!CheckRoutingPointLaneType(lane)) {
    return false;
  }
  return true;
}

bool MapService::CheckRoutingPointLaneType(LaneInfoConstPtr lane) const {
  if (lane->lane().type() != Lane::CITY_DRIVING &&
      lane->lane().type() != Lane::BIKING &&
      lane->lane().type() != Lane::SHARED) {
    AERROR
        << "Failed to construct LaneWayPoint for RoutingRequest: Expected lane "
        << lane->id().id() << " to be CITY_DRIVING, BIKING, SHARED, but was "
        << apollo::hdmap::Lane::LaneType_Name(lane->lane().type());
    return false;
  }
  return true;
}

bool MapService::GetStartPoint(apollo::common::PointENU *start_point) const {
  // Start from origin to find a lane from the map.
  double s, l;
  LaneInfoConstPtr lane;
  if (!GetNearestLane(0.0, 0.0, &lane, &s, &l)) {
    return false;
  }

  *start_point = lane->GetSmoothPoint(0.0);
  return true;
}

bool MapService::CreatePathsFromRouting(const RoutingResponse &routing,
                                        std::vector<Path> *paths) const {
  if (routing.road().empty()) {
    return false;
  }

  for (const auto &road : routing.road()) {
    for (const auto &passage_region : road.passage()) {
      // Each passage region in a road forms a path
      if (!AddPathFromPassageRegion(passage_region, paths)) {
        return false;
      }
    }
  }
  return true;
}

bool MapService::AddPathFromPassageRegion(
    const routing::Passage &passage_region, std::vector<Path> *paths) const {
  if (!MapReady()) {
    return false;
  }
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  std::vector<MapPathPoint> path_points;
  for (const auto &segment : passage_region.segment()) {
    auto lane_ptr = HDMap()->GetLaneById(hdmap::MakeMapId(segment.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << segment.id();
      return false;
    }
    if (segment.start_s() >= segment.end_s()) {
      continue;
    }
    auto points = MapPathPoint::GetPointsFromLane(lane_ptr, segment.start_s(),
                                                  segment.end_s());
    path_points.insert(path_points.end(), points.begin(), points.end());
  }
  // Remove duplicate path points that are too close to each other to avoid init
  // Path failure
  MapPathPoint::RemoveDuplicates(&path_points);

  if (path_points.size() < 2) {
    return false;
  }
  paths->emplace_back(Path(std::move(path_points)));

  return true;
}

size_t MapService::CalculateMapHash(const MapElementIds &ids) const {
  static std::hash<std::string> hash_function;
  return hash_function(ids.DebugString());
}

double MapService::GetLaneHeading(const std::string &id_str, double s) {
  auto *hdmap = HDMap();
  CHECK(hdmap) << "Failed to get hdmap";

  Id id;
  id.set_id(id_str);
  LaneInfoConstPtr lane_ptr = hdmap->GetLaneById(id);
  if (lane_ptr != nullptr) {
    return lane_ptr->Heading(s);
  }
  return 0.0;
}

std::string MapService::GetParkingSpaceId(const double x,
                                          const double y) const {
  PointENU point;
  point.set_x(x);
  point.set_y(y);
  static constexpr double kSearchRadius = 3.0;
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;

  // Find parking spaces nearby
  if (HDMap()->GetParkingSpaces(point, kSearchRadius, &parking_spaces) != 0) {
    AERROR << "Fail to get parking space from sim_map.";
    return "-1";
  }

  // Determine whether the point is in a neighboring parking space
  for (const auto &parking_sapce : parking_spaces) {
    if (!parking_sapce->polygon().is_convex()) {
      AERROR << "The parking space information is incorrect and is not a "
                "convex hull.";
      return "-1";
    }
    apollo::common::math::Vec2d checked_point(x, y);
    if (parking_sapce->polygon().IsPointIn(checked_point)) {
      return parking_sapce->id().id();
    }
  }
  return "-1";
}

}  // namespace dreamview
}  // namespace apollo
