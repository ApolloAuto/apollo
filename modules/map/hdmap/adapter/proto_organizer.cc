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
#include "modules/map/hdmap/adapter/proto_organizer.h"

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/math/polygon2d.h"

namespace {

std::string create_stop_line_id() {
  static int stop_line_id = 0;
  ++stop_line_id;
  return "stop_line_" + std::to_string(stop_line_id);
}

std::string create_overlap_id() {
  static int count = 0;
  ++count;
  return "overlap_" + std::to_string(count);
}

void points_from_curve(const ::apollo::hdmap::Curve &input_curve,
                       std::vector< apollo::common::math::Vec2d> *points) {
  CHECK_NOTNULL(points);
  points->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    }
  }
}

}  // namespace

namespace apollo {
namespace hdmap {
namespace adapter {

void ProtoOrganizer::GetRoadElements(std::vector<RoadInternal>* roads) {
  for (auto& road_internal : *roads) {
    // lanes
    for (auto& section_internal : road_internal.sections) {
      for (auto& lane_internal : section_internal.lanes) {
        std::string lane_id = lane_internal.lane.id().id();
        _proto_data.pb_lanes[lane_id] = lane_internal.lane;
        section_internal.section.add_lane_id()->set_id(lane_id);
      }
      (*road_internal.road.add_section()) = section_internal.section;
      _proto_data.pb_roads[road_internal.id] = road_internal.road;
    }
    // crosswalks
    for (auto& crosswalk : road_internal.crosswalks) {
      _proto_data.pb_crosswalks[crosswalk.id().id()] = crosswalk;
    }
    // clear areas
    for (auto& clear_area : road_internal.clear_areas) {
      _proto_data.pb_clear_areas[clear_area.id().id()] = clear_area;
    }
    // speed_bump
    for (auto& speed_bump : road_internal.speed_bumps) {
      _proto_data.pb_speed_bumps[speed_bump.id().id()] = speed_bump;
    }
    // stop lines
    for (auto& stop_line_internal : road_internal.stop_lines) {
      _proto_data.pb_stop_lines[stop_line_internal.id] = stop_line_internal;
    }
    // traffic_lights
    for (auto& traffic_light_internal : road_internal.traffic_lights) {
      auto& traffic_light = traffic_light_internal.traffic_light;
      for (auto stop_line_id : traffic_light_internal.stop_line_ids) {
        CHECK(_proto_data.pb_stop_lines.count(stop_line_id) > 0);
        auto& stop_line_curve = _proto_data.pb_stop_lines[stop_line_id].curve;
        (*traffic_light.add_stop_line()) = stop_line_curve;
      }
      _proto_data.pb_signals[traffic_light.id().id()] = traffic_light;
    }
    // stop signs
    for (auto& stop_sign_internal : road_internal.stop_signs) {
      auto& stop_sign = stop_sign_internal.stop_sign;
      for (auto stop_line_id : stop_sign_internal.stop_line_ids) {
        CHECK(_proto_data.pb_stop_lines.count(stop_line_id) > 0);
        auto& stop_line_curve = _proto_data.pb_stop_lines[stop_line_id].curve;
        (*stop_sign.add_stop_line()) = stop_line_curve;
      }
      _proto_data.pb_stop_signs[stop_sign.id().id()] = stop_sign;
    }
    // yield signs
    for (auto& yield_sign_internal : road_internal.yield_signs) {
      auto& yield_sign = yield_sign_internal.yield_sign;
      for (auto stop_line_id : yield_sign_internal.stop_line_ids) {
        CHECK(_proto_data.pb_stop_lines.count(stop_line_id) > 0);
        auto& stop_line_curve = _proto_data.pb_stop_lines[stop_line_id].curve;
        (*yield_sign.add_stop_line()) = stop_line_curve;
      }
      _proto_data.pb_yield_signs[yield_sign.id().id()] = yield_sign;
    }
  }
}

void ProtoOrganizer::GetJunctionElements(
    const std::vector<JunctionInternal>& junctions) {
  for (auto& junction_internal : junctions) {
    std::string junction_id = junction_internal.junction.id().id();
    _proto_data.pb_junctions[junction_id] = junction_internal.junction;
  }
}

void ProtoOrganizer::GetOverlapElements(
    const std::vector<RoadInternal>& roads,
    const std::vector<JunctionInternal>& junctions) {
  std::unordered_map<std::string, OverlapWithLane> lane_2_lane_overlaps;
  for (auto& road_internal : roads) {
    for (auto& road_section : road_internal.sections) {
      for (auto& lane_internal : road_section.lanes) {
        for (auto& lane_2_lane_overlap : lane_internal.overlap_lanes) {
          lane_2_lane_overlaps[lane_2_lane_overlap.object_id] =
                                                      lane_2_lane_overlap;
        }
      }
    }
  }

  std::unordered_set<std::string> close_table;
  // overlap
  for (auto& road_internal : roads) {
    for (auto& road_section : road_internal.sections) {
      for (auto& lane_internal : road_section.lanes) {
        std::string lane_id = lane_internal.lane.id().id();
        for (auto& overlap_object : lane_internal.overlap_objects) {
          std::string object_id = overlap_object.object_id;
          if (_proto_data.pb_crosswalks.count(object_id) <= 0
              && _proto_data.pb_clear_areas.count(object_id) <= 0
              && _proto_data.pb_speed_bumps.count(object_id) <= 0) {
                continue;
          }
          PbOverlap overlap;
          std::string overlap_id = create_overlap_id();
          _proto_data.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
          overlap.mutable_id()->set_id(overlap_id);
          PbObjectOverlapInfo* object_overlap = overlap.add_object();
          object_overlap->mutable_id()->set_id(lane_id);
          object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                  overlap_object.start_s);
          object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                  overlap_object.end_s);
          CHECK(_proto_data.pb_lanes.count(lane_id) > 0);
          object_overlap = overlap.add_object();
          object_overlap->mutable_id()->set_id(object_id);
          if (_proto_data.pb_crosswalks.count(object_id) > 0) {
            _proto_data.pb_crosswalks[object_id].add_overlap_id()
                    ->set_id(overlap_id);
            object_overlap->mutable_crosswalk_overlap_info();
          } else if (_proto_data.pb_clear_areas.count(object_id) > 0) {
            object_overlap->mutable_clear_area_overlap_info();
            _proto_data.pb_clear_areas[object_id].add_overlap_id()
                    ->set_id(overlap_id);
          } else if (_proto_data.pb_speed_bumps.count(object_id)) {
            object_overlap->mutable_speed_bump_overlap_info();
            _proto_data.pb_speed_bumps[object_id].add_overlap_id()
                    ->set_id(overlap_id);
          } else {
            assert(0);
          }
          _proto_data.pb_overlaps[overlap_id] = overlap;
        }
        for (auto& overlap_signal : lane_internal.overlap_signals) {
          std::string object_id = overlap_signal.object_id;
          if (_proto_data.pb_signals.count(object_id) <= 0
            && _proto_data.pb_stop_signs.count(object_id) <= 0
            && _proto_data.pb_yield_signs.count(object_id) <= 0) {
              std::cout << "cannot find signal object_id:" << object_id
                  << std::endl;
              continue;
            }
            PbOverlap overlap;
            std::string overlap_id = create_overlap_id();
            _proto_data.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
            overlap.mutable_id()->set_id(overlap_id);
            PbObjectOverlapInfo* object_overlap = overlap.add_object();
            object_overlap->mutable_id()->set_id(lane_id);
            object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                      overlap_signal.start_s);
            object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                      overlap_signal.end_s);
            CHECK(_proto_data.pb_lanes.count(lane_id) > 0);
            object_overlap = overlap.add_object();
            object_overlap->mutable_id()->set_id(object_id);
            if (_proto_data.pb_signals.count(object_id) > 0) {
              object_overlap->mutable_signal_overlap_info();
              _proto_data.pb_signals[object_id].add_overlap_id()->set_id(
                                                                overlap_id);
            } else if (_proto_data.pb_stop_signs.count(object_id) > 0) {
              object_overlap->mutable_stop_sign_overlap_info();
              _proto_data.pb_stop_signs[object_id].add_overlap_id()->set_id(
                                                                overlap_id);
            } else if (_proto_data.pb_yield_signs.count(object_id) > 0) {
              object_overlap->mutable_yield_sign_overlap_info();
              _proto_data.pb_yield_signs[object_id].add_overlap_id()
                              ->set_id(overlap_id);
            } else {
              assert(0);
            }
            _proto_data.pb_overlaps[overlap_id] = overlap;
          }
          for (auto& overlap_junction : lane_internal.overlap_junctions) {
            std::string object_id = overlap_junction.object_id;
            if (_proto_data.pb_junctions.count(object_id) <= 0) {
              std::cout << "cannot find junction object id:"
                    << object_id << std::endl;
              continue;
            }
            PbOverlap overlap;
            std::string overlap_id = create_overlap_id();
            _proto_data.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
            overlap.mutable_id()->set_id(overlap_id);
            PbObjectOverlapInfo* object_overlap = overlap.add_object();
            object_overlap->mutable_id()->set_id(lane_id);
            object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                overlap_junction.start_s);
            object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                overlap_junction.end_s);
            CHECK(_proto_data.pb_lanes.count(lane_id) > 0);
            object_overlap = overlap.add_object();
            object_overlap->mutable_id()->set_id(object_id);
            if (_proto_data.pb_junctions.count(object_id) > 0) {
              object_overlap->mutable_junction_overlap_info();
              _proto_data.pb_junctions[object_id].add_overlap_id()->set_id(
                                                                overlap_id);
            } else {
              assert(0);
            }
            _proto_data.pb_overlaps[overlap_id] = overlap;
          }
          for (auto& overlap_lane : lane_internal.overlap_lanes) {
            std::string object_id = overlap_lane.object_id;
            std::string unique_object_id = lane_id + "_" + object_id;
            if (close_table.count(unique_object_id) > 0) {
              continue;
            }
            unique_object_id = object_id + "_" + lane_id;
            if (close_table.count(unique_object_id) > 0) {
              continue;
            }
            close_table.insert(unique_object_id);
            PbOverlap overlap;
            std::string overlap_id = create_overlap_id();
            _proto_data.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
            overlap.mutable_id()->set_id(overlap_id);
            PbObjectOverlapInfo* object_overlap = overlap.add_object();
            object_overlap->mutable_id()->set_id(lane_id);
            object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                overlap_lane.start_s);
            object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                overlap_lane.end_s);
            CHECK(_proto_data.pb_lanes.count(object_id) > 0);
            object_overlap = overlap.add_object();
            if (_proto_data.pb_lanes.count(object_id) > 0) {
              CHECK(lane_2_lane_overlaps.count(object_id) == 1);
              auto& lane_2_lane_overlap = lane_2_lane_overlaps[object_id];
              object_overlap->mutable_lane_overlap_info()->set_start_s(
                  lane_2_lane_overlap.start_s);
              object_overlap->mutable_lane_overlap_info()->set_end_s(
                  lane_2_lane_overlap.end_s);
              object_overlap->mutable_lane_overlap_info()
                            ->set_is_merge(lane_2_lane_overlap.is_merge);
              _proto_data.pb_lanes[object_id].add_overlap_id()->set_id(
                                                              overlap_id);
            } else {
              assert(0);
            }
            _proto_data.pb_overlaps[overlap_id] = overlap;
          }
      }
    }
  }

  for (auto& junction_internal : junctions) {
    std::string junction_id = junction_internal.junction.id().id();
    for (auto& overlap_junction : junction_internal.overlap_with_junctions) {
      PbOverlap overlap;
      std::string overlap_id = create_overlap_id();
      _proto_data.pb_junctions[junction_id].add_overlap_id()->set_id(
                                                                overlap_id);
      overlap.mutable_id()->set_id(overlap_id);
      PbObjectOverlapInfo* object_overlap = overlap.add_object();
      object_overlap->mutable_id()->set_id(junction_id);
      std::string object_id = overlap_junction.object_id;
      object_overlap = overlap.add_object();
      object_overlap->mutable_id()->set_id(object_id);
      if (_proto_data.pb_crosswalks.count(object_id) > 0) {
        object_overlap->mutable_crosswalk_overlap_info();
        _proto_data.pb_crosswalks[object_id].add_overlap_id()->set_id(
                                                          overlap_id);
      } else if (_proto_data.pb_clear_areas.count(object_id) > 0) {
        object_overlap->mutable_clear_area_overlap_info();
        _proto_data.pb_clear_areas[object_id].add_overlap_id()->set_id(
                                                          overlap_id);
      } else {
        continue;
      }
      _proto_data.pb_overlaps[overlap_id] = overlap;
    }
  }
}

void ProtoOrganizer::OutputData(apollo::hdmap::Map* pb_map) {
  int lane_size = 0;
  int road_size = 0;
  int crosswalk_size = 0;
  int clear_area_size = 0;
  int speed_bump_size = 0;
  int signal_size = 0;
  int stop_sign_size = 0;
  int yield_sign_size = 0;
  int overlap_size = 0;
  int junction_size = 0;
  for (auto& road_pair : _proto_data.pb_roads) {
    ++road_size;
    *(pb_map->add_road()) = road_pair.second;
  }
  for (auto& lane_pair : _proto_data.pb_lanes) {
    ++lane_size;
    *(pb_map->add_lane()) = lane_pair.second;
  }
  for (auto& crosswalk_pair : _proto_data.pb_crosswalks) {
    ++crosswalk_size;
    *(pb_map->add_crosswalk()) = crosswalk_pair.second;
  }
  for (auto& clear_area_pair : _proto_data.pb_clear_areas) {
    ++clear_area_size;
    *(pb_map->add_clear_area()) = clear_area_pair.second;
  }
  for (auto& speed_bump_pair : _proto_data.pb_speed_bumps) {
    ++speed_bump_size;
    *(pb_map->add_speed_bump()) = speed_bump_pair.second;
  }
  for (auto& signal_pair : _proto_data.pb_signals) {
    ++signal_size;
    *(pb_map->add_signal()) = signal_pair.second;
  }
  for (auto& stop_sign_pair : _proto_data.pb_stop_signs) {
    ++stop_sign_size;
    *(pb_map->add_stop_sign()) = stop_sign_pair.second;
  }
  for (auto& yield_sign_pair : _proto_data.pb_yield_signs) {
    ++yield_sign_size;
    *(pb_map->add_yield()) = yield_sign_pair.second;
  }
  for (auto& junction_pair : _proto_data.pb_junctions) {
    ++junction_size;
    *(pb_map->add_junction()) = junction_pair.second;
  }
  for (auto& overlap_pair : _proto_data.pb_overlaps) {
    ++overlap_size;
    *(pb_map->add_overlap()) = overlap_pair.second;
  }
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
