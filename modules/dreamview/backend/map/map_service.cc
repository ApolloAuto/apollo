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

#include "modules/common/util/string_util.h"

namespace apollo {
namespace dreamview {

using ::apollo::common::PointENU;
using ::apollo::hdmap::Map;
using ::apollo::hdmap::Id;
using ::apollo::hdmap::LaneInfoConstPtr;
using ::apollo::hdmap::CrosswalkInfoConstPtr;
using ::apollo::hdmap::JunctionInfoConstPtr;
using ::apollo::hdmap::SignalInfoConstPtr;
using ::apollo::hdmap::StopSignInfoConstPtr;
using ::apollo::hdmap::YieldSignInfoConstPtr;

namespace {

template <typename MapElementInfoConstPtr>
void ExtractIds(const std::vector<MapElementInfoConstPtr> &items,
                std::vector<std::string> *ids) {
  ids->reserve(items.size());
  for (const auto &item : items) {
    ids->push_back(item->id().id());
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

template <typename MapElementInfoConstPtr>
void ExtractOverlapIds(const std::vector<MapElementInfoConstPtr> &items,
                       std::vector<std::string> *ids) {
  for (const auto &item : items) {
    for (auto &overlap_id : item->signal().overlap_id()) {
      ids->push_back(overlap_id.id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractStringVectorFromJson(const nlohmann::json &json_object,
                                 const std::string key,
                                 std::vector<std::string> *result) {
  auto iter = json_object.find(key);
  if (iter != json_object.end()) {
    result->reserve(iter->size());
    for (size_t i = 0; i < iter->size(); ++i) {
      result->push_back((*iter)[i]);
    }
  }
}

}  // namespace

MapElementIds::MapElementIds()
    : lane(),
      crosswalk(),
      junction(),
      signal(),
      stop_sign(),
      yield(),
      overlap() {}

MapElementIds::MapElementIds(const nlohmann::json &json_object)
    : MapElementIds() {
  ExtractStringVectorFromJson(json_object, "lane", &lane);
  ExtractStringVectorFromJson(json_object, "crosswalk", &crosswalk);
  ExtractStringVectorFromJson(json_object, "junction", &junction);
  ExtractStringVectorFromJson(json_object, "signal", &signal);
  ExtractStringVectorFromJson(json_object, "stopSign", &stop_sign);
  ExtractStringVectorFromJson(json_object, "yield", &yield);
  ExtractStringVectorFromJson(json_object, "overlap", &overlap);
}

size_t MapElementIds::Hash() const {
  static std::hash<std::string> hash_function;
  const std::string text = apollo::common::util::StrCat(
      apollo::common::util::PrintIter(lane, ""),
      apollo::common::util::PrintIter(crosswalk, ""),
      apollo::common::util::PrintIter(junction, ""),
      apollo::common::util::PrintIter(signal, ""),
      apollo::common::util::PrintIter(stop_sign, ""),
      apollo::common::util::PrintIter(yield, ""),
      apollo::common::util::PrintIter(overlap, ""));
  return hash_function(text);
}

nlohmann::json MapElementIds::Json() const {
  nlohmann::json result;
  result["lane"] = lane;
  result["crosswalk"] = crosswalk;
  result["junction"] = junction;
  result["signal"] = signal;
  result["stopSign"] = stop_sign;
  result["yield"] = yield;
  result["overlap"] = overlap;
  return result;
}

MapService::MapService(const std::string &map_filename) {
  if (hdmap_.load_map_from_file(map_filename)) {
    AFATAL << "Failed to load map: " << map_filename;
  }
  AINFO << "HDMap loaded, Map: " << map_filename;
}

MapElementIds MapService::CollectMapElements(const PointENU &point,
                                             double radius) const {
  MapElementIds result;

  std::vector<LaneInfoConstPtr> lanes;
  hdmap_.get_lanes(point, radius, &lanes);

  ExtractIds(lanes, &result.lane);

  std::vector<CrosswalkInfoConstPtr> crosswalks;
  hdmap_.get_crosswalks(point, radius, &crosswalks);
  ExtractIds(crosswalks, &result.crosswalk);

  std::vector<JunctionInfoConstPtr> junctions;
  hdmap_.get_junctions(point, radius, &junctions);
  ExtractIds(junctions, &result.junction);

  std::vector<SignalInfoConstPtr> signals;
  hdmap_.get_signals(point, radius, &signals);
  ExtractIds(signals, &result.signal);
  ExtractOverlapIds(signals, &result.overlap);

  std::vector<StopSignInfoConstPtr> stop_signs;
  hdmap_.get_stop_signs(point, radius, &stop_signs);
  ExtractIds(stop_signs, &result.stop_sign);

  std::vector<YieldSignInfoConstPtr> yield_signs;
  hdmap_.get_yield_signs(point, radius, &yield_signs);
  ExtractIds(yield_signs, &result.yield);

  return result;
}

Map MapService::RetrieveMapElements(const MapElementIds &ids) const {
  Map result;
  Id map_id;

  for (const auto &id : ids.lane) {
    map_id.set_id(id);
    auto element = hdmap_.get_lane_by_id(map_id);
    if (element) {
      *result.add_lane() = element->lane();
    }
  }

  for (const auto &id : ids.crosswalk) {
    map_id.set_id(id);
    auto element = hdmap_.get_crosswalk_by_id(map_id);
    if (element) {
      *result.add_crosswalk() = element->crosswalk();
    }
  }

  for (const auto &id : ids.junction) {
    map_id.set_id(id);
    auto element = hdmap_.get_junction_by_id(map_id);
    if (element) {
      *result.add_junction() = element->junction();
    }
  }

  for (const auto &id : ids.signal) {
    map_id.set_id(id);
    auto element = hdmap_.get_signal_by_id(map_id);
    if (element) {
      *result.add_signal() = element->signal();
    }
  }

  for (const auto &id : ids.stop_sign) {
    map_id.set_id(id);
    auto element = hdmap_.get_stop_sign_by_id(map_id);
    if (element) {
      *result.add_stop_sign() = element->stop_sign();
    }
  }

  for (const auto &id : ids.yield) {
    map_id.set_id(id);
    auto element = hdmap_.get_yield_sign_by_id(map_id);
    if (element) {
      *result.add_yield() = element->yield_sign();
    }
  }

  for (const auto &id : ids.overlap) {
    map_id.set_id(id);
    auto element = hdmap_.get_overlap_by_id(map_id);
    if (element) {
      *result.add_overlap() = element->overlap();
    }
  }

  return result;
}

}  // namespace dreamview
}  // namespace apollo
