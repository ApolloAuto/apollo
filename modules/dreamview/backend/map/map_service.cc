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

#include "modules/dreamview/backend/map/map_service.h"

#include <algorithm>
#include <fstream>

#include "modules/common/util/json_util.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

using apollo::common::PointENU;
using apollo::common::util::JsonUtil;
using apollo::hdmap::ClearAreaInfoConstPtr;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::Id;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::Map;
using apollo::hdmap::Path;
using apollo::hdmap::PncMap;
using apollo::hdmap::RoadInfoConstPtr;
using apollo::hdmap::RouteSegments;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::hdmap::SimMapFile;
using apollo::hdmap::StopSignInfoConstPtr;
using apollo::hdmap::YieldSignInfoConstPtr;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using google::protobuf::RepeatedPtrField;

namespace {

template <typename MapElementInfoConstPtr>
void ExtractIds(const std::vector<MapElementInfoConstPtr> &items,
                RepeatedPtrField<std::string> *ids) {
  ids->Reserve(items.size());
  for (const auto &item : items) {
    ids->Add()->assign(item->id().id());
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractOverlapIds(const std::vector<SignalInfoConstPtr> &items,
                       RepeatedPtrField<std::string> *ids) {
  for (const auto &item : items) {
    for (auto &overlap_id : item->signal().overlap_id()) {
      ids->Add()->assign(overlap_id.id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractOverlapIds(const std::vector<StopSignInfoConstPtr> &items,
                       RepeatedPtrField<std::string> *ids) {
  for (const auto &item : items) {
    for (auto &overlap_id : item->stop_sign().overlap_id()) {
      ids->Add()->assign(overlap_id.id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractRoadAndLaneIds(const std::vector<LaneInfoConstPtr> &lanes,
                           RepeatedPtrField<std::string> *lane_ids,
                           RepeatedPtrField<std::string> *road_ids) {
  lane_ids->Reserve(lanes.size());
  road_ids->Reserve(lanes.size());

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
  UpdateOffsets();
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

  std::vector<SignalInfoConstPtr> signals;
  if (SimMap()->GetSignals(point, radius, &signals) != 0) {
    AERROR << "Failed to get signals from sim_map.";
  }

  ExtractIds(signals, ids->mutable_signal());
  ExtractOverlapIds(signals, ids->mutable_overlap());

  std::vector<StopSignInfoConstPtr> stop_signs;
  if (SimMap()->GetStopSigns(point, radius, &stop_signs) != 0) {
    AERROR << "Failed to get stop signs from sim_map.";
  }
  ExtractIds(stop_signs, ids->mutable_stop_sign());
  ExtractOverlapIds(stop_signs, ids->mutable_overlap());

  std::vector<YieldSignInfoConstPtr> yield_signs;
  if (SimMap()->GetYieldSigns(point, radius, &yield_signs) != 0) {
    AERROR << "Failed to get yield signs from sim_map.";
  }
  ExtractIds(yield_signs, ids->mutable_yield());
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

  for (const auto &id : ids.overlap()) {
    map_id.set_id(id);
    auto element = SimMap()->GetOverlapById(map_id);
    if (element) {
      *result.add_overlap() = element->overlap();
    }
  }

  return result;
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

  laneWayPoint->set_id(lane->id().id());
  laneWayPoint->set_s(s);
  auto *pose = laneWayPoint->mutable_pose();
  pose->set_x(x);
  pose->set_y(y);

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

  RouteSegments segments;
  for (const auto &segment : passage_region.segment()) {
    auto lane_ptr = HDMap()->GetLaneById(hdmap::MakeMapId(segment.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << segment.id();
      return false;
    }
    segments.emplace_back(lane_ptr, segment.start_s(), segment.end_s());
  }

  paths->emplace_back();
  if (!PncMap::CreatePathFromLaneSegments(segments, &paths->back())) {
    return false;
  }

  return true;
}

size_t MapService::CalculateMapHash(const MapElementIds &ids) const {
  static std::hash<std::string> hash_function;
  return hash_function(ids.DebugString());
}

}  // namespace dreamview
}  // namespace apollo
