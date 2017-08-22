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

#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"

namespace {
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

  if (apollo::common::util::EndWith(map_filename, ".xml")) {
    if (!apollo::hdmap::adapter::OpendriveAdapter::LoadData(map_filename,
                                                            &_map)) {
      return -1;
    }
  } else {
    using apollo::common::util::GetProtoFromFile;
    if (!GetProtoFromFile(map_filename, &_map)) {
      return -1;
    }
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
    _crosswalk_table[crosswalk.id().id()].reset(new CrosswalkInfo(crosswalk));
  }
  for (const auto& stop_sign : _map.stop_sign()) {
    _stop_sign_table[stop_sign.id().id()].reset(new StopSignInfo(stop_sign));
  }
  for (const auto& yield_sign : _map.yield()) {
    _yield_sign_table[yield_sign.id().id()].reset(
                                                new YieldSignInfo(yield_sign));
  }
  for (const auto& overlap : _map.overlap()) {
    _overlap_table[overlap.id().id()].reset(new OverlapInfo(overlap));
  }

  for (const auto& road : _map.road()) {
    _road_table[road.id().id()].reset(new RoadInfo(road));
  }

  for (const auto& road_ptr_pair : _road_table) {
      const auto& road_id = road_ptr_pair.second->id();
      for (size_t i = 0; i < road_ptr_pair.second->sections().size(); ++i) {
          const auto& road_section = road_ptr_pair.second->sections()[i];
          const auto& section_id = road_section.id();
          for (int j = 0; j < road_section.lane_id_size(); ++j) {
              const auto& lane_id = road_section.lane_id(j);
              _lane_table[lane_id.id()]->set_road_id(road_id);
              _lane_table[lane_id.id()]->set_section_id(section_id);
          }
      }
  }
  for (const auto& lane_ptr_pair : _lane_table) {
      lane_ptr_pair.second->post_process(*this);
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

RoadInfoConstPtr HDMapImpl::get_road_by_id(const apollo::hdmap::Id& id) const {
  RoadTable::const_iterator it = _road_table.find(id.id());
  return it != _road_table.end() ? it->second : nullptr;
}

int HDMapImpl::get_lanes(const apollo::common::PointENU& point,
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

int HDMapImpl::get_roads(const apollo::common::PointENU& point,
                double distance,
                std::vector<RoadInfoConstPtr>* roads) const {
  return get_roads({point.x(), point.y()}, distance, roads);
}

int HDMapImpl::get_roads(const apollo::common::math::Vec2d& point,
                double distance,
                std::vector<RoadInfoConstPtr>* roads) const {
    std::vector<LaneInfoConstPtr> lanes;
    if (get_lanes(point, distance, &lanes) != 0) {
        return -1;
    }
    std::unordered_set<std::string> road_ids;
    for (auto& lane : lanes) {
        road_ids.insert(lane->road_id().id());
    }

    for (auto& road_id : road_ids) {
        RoadInfoConstPtr road = get_road_by_id(create_hdmap_id(road_id));
        CHECK_NOTNULL(road);
        roads->push_back(road);
    }

    return 0;
}

int HDMapImpl::get_junctions(const apollo::common::PointENU& point,
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

int HDMapImpl::get_signals(const apollo::common::PointENU& point,
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

int HDMapImpl::get_crosswalks(const apollo::common::PointENU& point,
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

int HDMapImpl::get_stop_signs(const apollo::common::PointENU& point,
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

int HDMapImpl::get_yield_signs(const apollo::common::PointENU& point,
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

int HDMapImpl::get_nearest_lane(const apollo::common::PointENU& point,
                LaneInfoConstPtr* nearest_lane,
                double* nearest_s,
                double* nearest_l) const {
  return get_nearest_lane(apollo::common::math::Vec2d(point.x(), point.y()),
            nearest_lane, nearest_s, nearest_l);
}

int HDMapImpl::get_nearest_lane(const apollo::common::math::Vec2d &point,
                            LaneInfoConstPtr* nearest_lane,
                            double *nearest_s, double *nearest_l) const {
  CHECK(nearest_lane != nullptr);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);
  const auto *segment_object = _lane_segment_kdtree->GetNearestObject(point);
  if (segment_object == nullptr) {
      return -1;
  }
  const apollo::hdmap::Id& lane_id = segment_object->object()->id();
  *nearest_lane = get_lane_by_id(lane_id);
  CHECK(*nearest_lane != nullptr);
  const int id = segment_object->id();
  const auto &segment = (*nearest_lane)->segments()[id];
  apollo::common::math::Vec2d nearest_pt;
  segment.DistanceTo(point, &nearest_pt);
  *nearest_s = (*nearest_lane)->accumulate_s()[id] +
              nearest_pt.DistanceTo(segment.start());
  *nearest_l = segment.unit_direction().CrossProd(point - segment.start());

  return 0;
}

int HDMapImpl::get_nearest_lane_with_heading(const apollo::common::PointENU& point,
                                  const double distance,
                                  const double central_heading,
                                  const double max_heading_difference,
                                  LaneInfoConstPtr* nearest_lane,
                                  double* nearest_s,
                                  double* nearest_l) const {
  return get_nearest_lane_with_heading(apollo::common::math::Vec2d(point.x(),
    point.y()), distance, central_heading, max_heading_difference,
    nearest_lane, nearest_s, nearest_l);
}

int HDMapImpl::get_nearest_lane_with_heading(
                                  const apollo::common::math::Vec2d& point,
                                  const double distance,
                                  const double central_heading,
                                  const double max_heading_difference,
                                  LaneInfoConstPtr* nearest_lane,
                                  double* nearest_s,
                                  double* nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);

  std::vector<LaneInfoConstPtr> lanes;
  if (get_lanes_with_heading(point, distance, central_heading,
                             max_heading_difference, &lanes) != 0) {
    return -1;
  }

  double s = 0;
  size_t s_index = 0;
  apollo::common::math::Vec2d map_point;
  double min_distance = distance;
  for (const auto &lane : lanes) {
    double s_offset = 0.0;
    int s_offset_index = 0;
    double distance = lane->distance_to(point, &map_point, &s_offset,
                                        &s_offset_index);
    if (distance < min_distance) {
      min_distance = distance;
      *nearest_lane = lane;
      s = s_offset;
      s_index = s_offset_index;
    }
  }

  if (*nearest_lane == nullptr) {
    return -1;
  }

  *nearest_s = s;
  int segment_index = s_index >= (*nearest_lane)->segments().size() ?
                            (*nearest_lane)->segments().size() - 1 : s_index;
  const auto& segment_2d = (*nearest_lane)->segments()[segment_index];
  *nearest_l = segment_2d.unit_direction().CrossProd(
                                                    point - segment_2d.start());

  return 0;
}

int HDMapImpl::get_lanes_with_heading(const apollo::common::PointENU& point,
                            const double distance,
                            const double central_heading,
                            const double max_heading_difference,
                            std::vector<LaneInfoConstPtr>* lanes) const {
  return get_lanes_with_heading(apollo::common::math::Vec2d(point.x(),
        point.y()), distance, central_heading, max_heading_difference, lanes);
}

int HDMapImpl::get_lanes_with_heading(const apollo::common::math::Vec2d &point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                std::vector<LaneInfoConstPtr> *lanes) const {
  CHECK_NOTNULL(lanes);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = get_lanes(point, distance, &all_lanes);
  if (status < 0 || all_lanes.size() <= 0) {
    return -1;
  }

  lanes->clear();
  for (auto& lane : all_lanes) {
    apollo::common::math::Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->distance_to(point, &proj_pt,
                                    &s_offset, &s_offset_index);
    if (dis <= distance) {
      double heading_diff =
                fabs(lane->headings()[s_offset_index] - central_heading);
      if (fabs(apollo::common::math::NormalizeAngle(heading_diff))
                                                    <= max_heading_difference) {
        lanes->push_back(lane);
      }
    }
  }

  return 0;
}

int HDMapImpl::get_road_boundaries(const apollo::common::PointENU& point,
                        double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const {
  CHECK_NOTNULL(road_boundaries);
  CHECK_NOTNULL(junctions);

  road_boundaries->clear();
  junctions->clear();

  std::vector<LaneInfoConstPtr> lanes;
  if (get_lanes(point, radius, &lanes) != 0 || lanes.size() <= 0) {
        return -1;
  }

  std::unordered_set<std::string> junction_id_set;
  std::unordered_set<std::string> road_section_id_set;
  for (const auto& lane : lanes) {
    const auto road_id = lane->road_id();
    const auto section_id = lane->section_id();
    std::string unique_id = road_id.id();
    unique_id += section_id.id();
      if (road_section_id_set.count(unique_id) > 0) {
          continue;
      }
      road_section_id_set.insert(unique_id);
      const auto road_ptr = get_road_by_id(road_id);
      CHECK_NOTNULL(road_ptr);
      if (road_ptr->has_junction_id()) {
          const apollo::hdmap::Id junction_id = road_ptr->junction_id();
          if (junction_id_set.count(junction_id.id()) > 0) {
              continue;
          }
          junction_id_set.insert(junction_id.id());
          JunctionBoundaryPtr junction_boundary_ptr(new JunctionBoundary());
          junction_boundary_ptr->junction_info =
                                              get_junction_by_id(junction_id);
          CHECK_NOTNULL(junction_boundary_ptr->junction_info);
          junctions->push_back(junction_boundary_ptr);
      } else {
          RoadROIBoundaryPtr road_boundary_ptr(new RoadROIBoundary());
          road_boundary_ptr->id = road_ptr->id();
          for (size_t i = 0; i < road_ptr->sections().size(); ++i) {
              const auto& section = road_ptr->sections()[i];
              if (section.id().id() == section_id.id()) {
                  road_boundary_ptr->road_boundaries.push_back(
                                                          section.boundary());
              }
          }
          road_boundaries->push_back(road_boundary_ptr);
      }
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
