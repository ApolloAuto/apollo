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

#include <fstream>
#include <iostream>
#include <unordered_set>

namespace {

bool read_map_from_bin_file(const std::string& file,
                            apollo::hdmap::Map* const map) {
    std::fstream input(file, std::ios::in | std::ios::binary);
    return map->ParseFromIstream(&input);
}

apollo::hdmap::Id create_hdmap_id(const std::string& string_id) {
    apollo::hdmap::Id id;
    id.set_id(string_id);
    return id;
}
}  // namespace

namespace apollo {
namespace hdmap {

int HDMapImpl::load_map_from_file(const std::string& map_filename) {
    clear();

    if (!read_map_from_bin_file(map_filename, &_map)) {
        return -1;
    }

    for (const auto& lane : _map.lane()) {
        _lane_table[lane.id().id()].reset(new LaneInfo(lane));
    }
    for (const auto& junction : _map.junction()) {
        _junction_table[junction.id().id()].reset(new JunctionInfo(junction));
    }
    for (const auto& signal : _map.signal()) {
        _signal_table[signal.id().id()].reset(new SignalInfo(signal));
    }
    for (const auto& crosswalk : _map.crosswalk()) {
        _crosswalk_table[crosswalk.id().id()].reset(
                                                new CrosswalkInfo(crosswalk));
    }
    for (const auto& stop_sign : _map.stop_sign()) {
        _stop_sign_table[stop_sign.id().id()].reset(
                                                new StopSignInfo(stop_sign));
    }
    for (const auto& yield_sign : _map.yield()) {
        _yield_sign_table[yield_sign.id().id()].reset(
                                                new YieldSignInfo(yield_sign));
    }
    for (const auto& overlap : _map.overlap()) {
        _overlap_table[overlap.id().id()].reset(new OverlapInfo(overlap));
    }

    build_lane_segment_kdtree();
    build_junction_polygon_kdtree();
    build_signal_segment_kdtree();
    build_crosswalk_polygon_kdtree();
    build_stop_sign_segment_kdtree();
    build_yield_sign_segment_kdtree();

    return 0;
}

LaneInfoConstPtr HDMapImpl::get_lane_by_id(const apollo::hdmap::Id& id) const {
    LaneTable::const_iterator it = _lane_table.find(id.id());
    return it != _lane_table.end() ? it->second : nullptr;
}

JunctionInfoConstPtr HDMapImpl::get_junction_by_id(
                                            const apollo::hdmap::Id& id) const {
    JunctionTable::const_iterator it = _junction_table.find(id.id());
    return it != _junction_table.end() ? it->second : nullptr;
}

SignalInfoConstPtr HDMapImpl::get_signal_by_id(
                                            const apollo::hdmap::Id& id) const {
    SignalTable::const_iterator it = _signal_table.find(id.id());
    return it != _signal_table.end() ? it->second : nullptr;
}

CrosswalkInfoConstPtr HDMapImpl::get_crosswalk_by_id(
                                            const apollo::hdmap::Id& id) const {
    CrosswalkTable::const_iterator it = _crosswalk_table.find(id.id());
    return it != _crosswalk_table.end() ? it->second : nullptr;
}

StopSignInfoConstPtr HDMapImpl::get_stop_sign_by_id(
                                            const apollo::hdmap::Id& id) const {
    StopSignTable::const_iterator it = _stop_sign_table.find(id.id());
    return it != _stop_sign_table.end() ? it->second : nullptr;
}

YieldSignInfoConstPtr HDMapImpl::get_yield_sign_by_id(
                                            const apollo::hdmap::Id& id) const {
    YieldSignTable::const_iterator it = _yield_sign_table.find(id.id());
    return it != _yield_sign_table.end() ? it->second : nullptr;
}

OverlapInfoConstPtr HDMapImpl::get_overlap_by_id(
                                            const apollo::hdmap::Id& id) const {
    OverlapTable::const_iterator it = _overlap_table.find(id.id());
    return it != _overlap_table.end() ? it->second : nullptr;
}

int HDMapImpl::get_lanes(const apollo::hdmap::Point& point,
                     double distance,
                     std::vector<LaneInfoConstPtr>* lanes) const {
  return get_lanes({point.x(), point.y()}, distance, lanes);
}

int HDMapImpl::get_lanes(const apollo::common::math::Vec2d &point,
                    double distance,
                    std::vector<LaneInfoConstPtr> *lanes) const {
  if (lanes == nullptr || _lane_segment_kdtree == nullptr) {
    return -1;
  }

  lanes->clear();
  std::vector<std::string> ids;
  const int status = search_objects(point, distance, *_lane_segment_kdtree,
                                    &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    lanes->emplace_back(get_lane_by_id(create_hdmap_id(id)));
  }
  return 0;
}

int HDMapImpl::get_junctions(const apollo::hdmap::Point& point,
                         double distance,
                         std::vector<JunctionInfoConstPtr>* junctions) const {
return get_junctions({point.x(), point.y()}, distance, junctions);
}

int HDMapImpl::get_junctions(const apollo::common::math::Vec2d& point,
                          double distance,
                          std::vector<JunctionInfoConstPtr>* junctions) const {
    if (junctions == nullptr || _junction_polygon_kdtree == nullptr) {
        return -1;
    }
    junctions->clear();
    std::vector<std::string> ids;
    const int status = search_objects(point, distance,
                                    *_junction_polygon_kdtree, &ids);
    if (status < 0) {
        return status;
    }
    for (const auto& id : ids) {
        junctions->emplace_back(get_junction_by_id(create_hdmap_id(id)));
    }
    return 0;
}

int HDMapImpl::get_signals(const apollo::hdmap::Point& point,
                       double distance,
                       std::vector<SignalInfoConstPtr>* signals) const {
    return get_signals({point.x(), point.y()}, distance, signals);
}

int HDMapImpl::get_signals(const apollo::common::math::Vec2d& point,
                        double distance,
                        std::vector<SignalInfoConstPtr>* signals) const {
    if (signals == nullptr || _signal_segment_kdtree == nullptr) {
        return -1;
    }
    signals->clear();
    std::vector<std::string> ids;
    const int status = search_objects(point, distance, *_signal_segment_kdtree,
                                    &ids);
    if (status < 0) {
        return status;
    }
    for (const auto& id : ids) {
        signals->emplace_back(get_signal_by_id(create_hdmap_id(id)));
    }
    return 0;
}

int HDMapImpl::get_crosswalks(const apollo::hdmap::Point& point,
                        double distance,
                        std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
    return get_crosswalks({point.x(), point.y()}, distance, crosswalks);
}

int HDMapImpl::get_crosswalks(const apollo::common::math::Vec2d& point,
                        double distance,
                        std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
    if (crosswalks == nullptr || _crosswalk_polygon_kdtree == nullptr) {
        return -1;
    }
    crosswalks->clear();
    std::vector<std::string> ids;
    const int status = search_objects(point, distance,
                                    *_crosswalk_polygon_kdtree, &ids);
    if (status < 0) {
        return status;
    }
    for (const auto& id : ids) {
        crosswalks->emplace_back(get_crosswalk_by_id(create_hdmap_id(id)));
    }
    return 0;
}

int HDMapImpl::get_stop_signs(const apollo::hdmap::Point& point,
                          double distance,
                          std::vector<StopSignInfoConstPtr>* stop_signs) const {
    return get_stop_signs({point.x(), point.y()}, distance, stop_signs);
}

int HDMapImpl::get_stop_signs(const apollo::common::math::Vec2d& point,
                        double distance,
                        std::vector<StopSignInfoConstPtr>* stop_signs) const {
    if (stop_signs == nullptr || _stop_sign_segment_kdtree == nullptr) {
        return -1;
    }
    stop_signs->clear();
    std::vector<std::string> ids;
    const int status = search_objects(point, distance,
                                    *_stop_sign_segment_kdtree, &ids);
    if (status < 0) {
        return status;
    }
    for (const auto& id : ids) {
        stop_signs->emplace_back(get_stop_sign_by_id(create_hdmap_id(id)));
    }
    return 0;
}

int HDMapImpl::get_yield_signs(const apollo::hdmap::Point& point,
                        double distance,
                        std::vector<YieldSignInfoConstPtr>* yield_signs) const {
    return get_yield_signs({point.x(), point.y()}, distance, yield_signs);
}

int HDMapImpl::get_yield_signs(const apollo::common::math::Vec2d& point,
                        double distance,
                        std::vector<YieldSignInfoConstPtr>* yield_signs) const {
    if (yield_signs == nullptr || _yield_sign_segment_kdtree == nullptr) {
        return -1;
    }
    yield_signs->clear();
    std::vector<std::string> ids;
    const int status = search_objects(point, distance,
                                    *_yield_sign_segment_kdtree, &ids);
    if (status < 0) {
        return status;
    }
    for (const auto& id : ids) {
        yield_signs->emplace_back(get_yield_sign_by_id(create_hdmap_id(id)));
    }

    return 0;
}

template<class Table, class BoxTable, class KDTree>
void HDMapImpl::build_segment_kdtree(const Table& table,
        const apollo::common::math::AABoxKDTreeParams& params,
        BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree) {
    box_table->clear();
    for (const auto& info_with_id : table) {
        const auto* info = info_with_id.second.get();
        for (size_t id = 0; id < info->segments().size(); ++id) {
            const auto& segment = info->segments()[id];
            box_table->emplace_back(
            apollo::common::math::AABox2d(segment.start(), segment.end()), info,
                                        &segment, id);
        }
    }
    kdtree->reset(new KDTree(*box_table, params));
}

template<class Table, class BoxTable, class KDTree>
void HDMapImpl::build_polygon_kdtree(const Table& table,
        const apollo::common::math::AABoxKDTreeParams& params,
        BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree) {
    box_table->clear();
    for (const auto& info_with_id : table) {
        const auto* info = info_with_id.second.get();
        const auto& polygon = info->polygon();
        box_table->emplace_back(polygon.AABoundingBox(), info, &polygon, 0);
    }
    kdtree->reset(new KDTree(*box_table, params));
}

void HDMapImpl::build_lane_segment_kdtree() {
    apollo::common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 16;
    build_segment_kdtree(_lane_table, params, &_lane_segment_boxes,
                        &_lane_segment_kdtree);
}

void HDMapImpl::build_junction_polygon_kdtree() {
    apollo::common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 1;
    build_polygon_kdtree(_junction_table, params, &_junction_polygon_boxes,
                        &_junction_polygon_kdtree);
}

void HDMapImpl::build_crosswalk_polygon_kdtree() {
    apollo::common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 1;
    build_polygon_kdtree(_crosswalk_table, params, &_crosswalk_polygon_boxes,
                        &_crosswalk_polygon_kdtree);
}

void HDMapImpl::build_signal_segment_kdtree() {
    apollo::common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 4;
    build_segment_kdtree(_signal_table, params, &_signal_segment_boxes,
                        &_signal_segment_kdtree);
}

void HDMapImpl::build_stop_sign_segment_kdtree() {
    apollo::common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 4;
    build_segment_kdtree(
        _stop_sign_table, params, &_stop_sign_segment_boxes,
        &_stop_sign_segment_kdtree);
}

void HDMapImpl::build_yield_sign_segment_kdtree() {
    apollo::common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 4;
    build_segment_kdtree(
        _yield_sign_table, params, &_yield_sign_segment_boxes,
        &_yield_sign_segment_kdtree);
}

template<class KDTree>
int HDMapImpl::search_objects(const apollo::common::math::Vec2d& center,
                            const double radius,
                            const KDTree& kdtree,
                            std::vector<std::string>* const results) {
    if (results == nullptr) {
        return -1;
    }
    auto objects = kdtree.GetObjects(center, radius);
    std::unordered_set<std::string> result_ids;
    for (const auto* object_ptr : objects) {
        result_ids.insert(object_ptr->object()->id().id());
    }
    *results = std::vector<std::string>(result_ids.begin(), result_ids.end());
    return 0;
}

int HDMapImpl::get_nearest_lane(const apollo::hdmap::Point& point,
                LaneInfoConstPtr* nearest_lane,
                double* nearest_s,
                double* nearest_l) {
    return get_nearest_lane(apollo::common::math::Vec2d(point.x(), point.y()),
                nearest_lane, nearest_s, nearest_l);
}

int HDMapImpl::get_nearest_lane(const apollo::common::math::Vec2d &point,
                            LaneInfoConstPtr* nearest_lane,
                            double *nearest_s, double *nearest_l) const {
    // CHECK(nearest_lane == nullptr);
    CHECK_NOTNULL(nearest_s);
    CHECK_NOTNULL(nearest_l);
    const auto *segment_object =
                _lane_segment_kdtree->GetNearestObject(point);
    if (segment_object == nullptr) {
        return -1;
    }
    const apollo::hdmap::Id& lane_id = segment_object->object()->id();
    *nearest_lane = get_lane_by_id(lane_id);
    CHECK(*nearest_lane != nullptr);
    // *nearest_lane = segment_object->object();
    const int id = segment_object->id();
    const auto &segment = (*nearest_lane)->segments()[id];
    apollo::common::math::Vec2d nearest_pt;
    segment.DistanceTo(point, &nearest_pt);
    *nearest_s = (*nearest_lane)->accumulate_s()[id] +
                nearest_pt.DistanceTo(segment.start());
    *nearest_l = segment.unit_direction().CrossProd(point - segment.start());

    return 0;
}

void HDMapImpl::clear() {
    _map.Clear();
    _lane_table.clear();
    _junction_table.clear();
    _signal_table.clear();
    _crosswalk_table.clear();
    _stop_sign_table.clear();
    _yield_sign_table.clear();
    _overlap_table.clear();
    _lane_segment_boxes.clear();
    _lane_segment_kdtree.reset(nullptr);
    _junction_polygon_boxes.clear();
    _junction_polygon_kdtree.reset(nullptr);
    _crosswalk_polygon_boxes.clear();
    _crosswalk_polygon_kdtree.reset(nullptr);
    _signal_segment_boxes.clear();
    _signal_segment_kdtree.reset(nullptr);
    _stop_sign_segment_boxes.clear();
    _stop_sign_segment_kdtree.reset(nullptr);
    _yield_sign_segment_boxes.clear();
    _yield_sign_segment_kdtree.reset(nullptr);
}

}  // namespace hdmap
}  // namespace apollo
