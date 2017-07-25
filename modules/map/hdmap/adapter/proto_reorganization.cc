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
#include "modules/map/hdmap/adapter/proto_reorganization.h"
#include <functional>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include "modules/common/math/vec2d.h"
#include "modules/common/math/polygon2d.h"
#include "glog/logging.h"

namespace {
std::vector<std::string> split(std::string str,
                            std::string pattern) {
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern;
    size_t size = str.size();
    for (size_t i = 0; i < size; i++) {
        pos = str.find(pattern, i);
        if (pos < size) {
            std::string s = str.substr(i, pos-i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}
std::string create_stop_line_id() {
    static int stop_line_id = 0;
    ++stop_line_id;
    return "stop_line_" + std::to_string(stop_line_id);
}
std::string get_road_id(const std::string& lane_id) {
    std::string tmp_lane_id = lane_id;
    auto split_ids = split(tmp_lane_id, "_");
    assert(split_ids.size() > 0);
    return split_ids[0];
}

std::string create_overlap_id() {
    static int count = 0;
    ++count;

    return "overlap_" + std::to_string(count);
}
void points_from_curve(const ::apollo::hdmap::Curve &input_curve,
                       std::vector< apollo::common::math::Vec2d> *points) {
    points->clear();

    for (const auto &curve : input_curve.segment()) {
        if (curve.has_line_segment()) {
            for (const auto &point : curve.line_segment().point()) {
                points->emplace_back(point.x(), point.y());
            }
        } else {
           assert(0);
        }
    }
}

}  // namespace

namespace apollo {
namespace hdmap {
namespace adapter {

ProtoOrganization::ProtoOrganization()
    : _pb_map(nullptr) {
}

ProtoOrganization::ProtoOrganization(::apollo::hdmap::Map* pb_map)
    : _pb_map(pb_map) {
    load_data();
}

ProtoOrganization::~ProtoOrganization() {
}

PbHeader ProtoOrganization::get_header() {
    return _pb_map->header();
}

void ProtoOrganization::load_data() {
    // int lane_size = 0;
    // int crosswalk_size = 0;
    // int clear_area_size = 0;
    // int speed_bump_size = 0;
    // int signal_size = 0;
    // int stop_sign_size = 0;
    // int yield_sign_size = 0;
    // int overlap_size = 0;
    // int junction_size = 0;

    _pb_map_data.header = _pb_map->header();
    for (int i = 0; i < _pb_map->crosswalk_size(); ++i) {
        // ++crosswalk_size;
        const PbCrosswalk& crosswalk = _pb_map->crosswalk(i);
        _pb_map_data.pb_crosswalks[crosswalk.id().id()] = crosswalk;
    }

    for (int i = 0; i < _pb_map->clear_area_size(); ++i) {
        // ++clear_area_size;
        const PbClearArea& clear_area = _pb_map->clear_area(i);
        _pb_map_data.pb_clear_areas[clear_area.id().id()] = clear_area;
    }

    for (int i = 0; i < _pb_map->speed_bump_size(); ++i) {
        // ++speed_bump_size;
        const PbSpeedBump& speed_bump = _pb_map->speed_bump(i);
        _pb_map_data.pb_speed_bumps[speed_bump.id().id()] = speed_bump;
    }

    for (int i = 0; i < _pb_map->junction_size(); ++i) {
        // ++junction_size;
        const PbJunction& junction = _pb_map->junction(i);
        _pb_map_data.pb_junctions[junction.id().id()] = junction;
    }

    for (int i = 0; i < _pb_map->lane_size(); ++i) {
        // ++lane_size;
        const PbLane& lane = _pb_map->lane(i);
        _pb_map_data.pb_lanes[lane.id().id()] = lane;

        // record junction to road map
        std::string lane_id = lane.id().id();
        if (lane.has_junction_id()) {
            _road_in_junctions[get_road_id(lane_id)].insert(
                                                    lane.junction_id().id());
        }
    }

    for (int i = 0; i < _pb_map->stop_sign_size(); ++i) {
        // ++stop_sign_size;
        const PbStopSign& stop_sign = _pb_map->stop_sign(i);
        _pb_map_data.pb_stop_signs[stop_sign.id().id()] = stop_sign;
    }

    for (int i = 0; i < _pb_map->yield_size(); ++i) {
        // ++yield_sign_size;
        const PbYieldSign& yield_sign = _pb_map->yield(i);
        _pb_map_data.pb_yield_signs[yield_sign.id().id()] = yield_sign;
    }

    for (int i = 0; i < _pb_map->signal_size(); ++i) {
        // ++signal_size;
        const PbSignal& signal = _pb_map->signal(i);
        _pb_map_data.pb_signals[signal.id().id()] = signal;
    }

    for (int i = 0; i < _pb_map->overlap_size(); ++i) {
        // ++overlap_size;
        auto& overlap = _pb_map->overlap(i);
        _pb_map_data.pb_overlaps[overlap.id().id()] = overlap;
    }

    // std::cout << "total: lane-" << lane_size
    //     << ";crosswalk-" << crosswalk_size
    //     << ";clear area-" << clear_area_size
    //     << ";speed bump-" << speed_bump_size
    //     << ";signal-" << signal_size
    //     << ";stop sign-" << stop_sign_size
    //     << ";yield sign-" << yield_sign_size
    //     << ";junction-" << junction_size
    //     << ";overlap-" << overlap_size
    //     << std::endl;
}

bool ProtoOrganization::is_road_in_junction(const std::string& road_id) {
    if (_road_in_junctions.count(road_id) <= 0) {
        return false;
    }

    assert(_road_in_junctions[road_id].size() == 1);

    return true;
}

std::string ProtoOrganization::get_road_junction_id(
                                                const std::string& road_id) {
    assert(_road_in_junctions.count(road_id) == 1);

    return *_road_in_junctions[road_id].begin();
}

std::unordered_map<std::string, Road> ProtoOrganization::get_roads() {
  std::unordered_map<std::string, Road> roads;
  for (int i = 0; i < _pb_map->lane_size(); ++i) {
      const PbLane& lane = _pb_map->lane(i);
      // somewhat trick
      std::string id = get_road_id(lane.id().id());
      Road& road = roads[id];
      road.id = id;
      if (is_road_in_junction(road.id)) {
          road.in_junction = true;
          road.junction_id = get_road_junction_id(road.id);
      }
      for (int j = 0; j < lane.predecessor_id_size(); ++j) {
          auto& precessor_id = lane.predecessor_id(j);
          std::string predecessor_road_id = get_road_id(precessor_id.id());
          if (!is_road_in_junction(predecessor_road_id)) {
              road.predecessor_ids.insert(predecessor_road_id);
          } else {
              road.junction_predecessor_ids.insert(
                                  get_road_junction_id(predecessor_road_id));
          }
      }
      for (int j = 0; j < lane.successor_id_size(); ++j) {
          auto& successor_id = lane.successor_id(j);
          std::string successor_road_id = get_road_id(successor_id.id());
          if (!is_road_in_junction(successor_road_id)) {
              road.successor_ids.insert(successor_road_id);
          } else {
              road.junction_successor_ids.insert(
                      get_road_junction_id(successor_road_id));
          }
      }
      road.lane_ids.insert(lane.id().id());
      LaneInternal lane_internal;
      lane_internal.lane = lane;
      for (int j = 0; j < lane.overlap_id_size(); ++j) {
          const PbID& lane_overlap_id = lane.overlap_id(j);
          auto& lane_overap = _pb_map_data.pb_overlaps[lane_overlap_id.id()];
          OverlapWithLane overlap_with_lane;
          for (int k = 0; k < lane_overap.object_size(); ++k) {
              const PbObjectOverlapInfo& overlap_obj = lane_overap.object(k);
              if (overlap_obj.id().id() == lane.id().id()) {
                  assert(overlap_obj.has_lane_overlap_info());
                  auto& lane_overap_info = overlap_obj.lane_overlap_info();
                  // record offset
                  overlap_with_lane.start_s = lane_overap_info.start_s();
                  overlap_with_lane.end_s = lane_overap_info.end_s();
                  overlap_with_lane.is_merge = lane_overap_info.is_merge();
              }
          }
          for (int k = 0; k < lane_overap.object_size(); ++k) {
              const PbObjectOverlapInfo& overlap_obj = lane_overap.object(k);
              if (overlap_obj.has_signal_overlap_info()) {
                  std::string signal_id = overlap_obj.id().id();
                  assert(_pb_map_data.pb_signals.count(signal_id) == 1);
                  auto& traffic_light = _pb_map_data.pb_signals[signal_id];
                  TrafficLightInternal signal_internal;
                  for (int m = 0; m < traffic_light.stop_line_size(); ++m) {
                      StopLineInternal stop_line;
                      stop_line.id = create_stop_line_id();
                      stop_line.curve = traffic_light.stop_line(m);
                      road.stop_lines.push_back(stop_line);
                      signal_internal.stop_line_ids.insert(stop_line.id);
                      if (stop_line_in_lane_boundary(stop_line.curve, lane)) {
                          overlap_with_lane.object_id = stop_line.id;
                          lane_internal.overlap_objects.push_back(
                                                          overlap_with_lane);
                      }
                  }
                  signal_internal.traffic_light = traffic_light;
                  road.traffic_lights.push_back(signal_internal);
                  overlap_with_lane.object_id = signal_id;
                  lane_internal.overlap_signals.push_back(overlap_with_lane);
              } else if (overlap_obj.has_stop_sign_overlap_info()) {
                  std::string stop_sign_id = overlap_obj.id().id();
                  assert(_pb_map_data.pb_stop_signs.count(stop_sign_id) == 1);
                  auto& stop_sign = _pb_map_data.pb_stop_signs[stop_sign_id];
                  StopSignInternal stop_sign_internal;
                  StopLineInternal stop_line;
                  stop_line.id = create_stop_line_id();
                  stop_line.curve = stop_sign.stop_line();
                  road.stop_lines.push_back(stop_line);
                  stop_sign_internal.stop_line_ids.insert(stop_line.id);
                  stop_sign_internal.id = stop_sign.id().id();
                  stop_sign_internal.stop_sign = stop_sign;
                  road.stop_signs.push_back(stop_sign_internal);
                  overlap_with_lane.object_id = stop_line.id;
                  lane_internal.overlap_objects.push_back(overlap_with_lane);
                  overlap_with_lane.object_id = stop_sign_internal.id;;
                  lane_internal.overlap_signals.push_back(overlap_with_lane);
              } else if (overlap_obj.has_yield_sign_overlap_info()) {
                  std::string yield_sign_id = overlap_obj.id().id();
                  assert(_pb_map_data.pb_yield_signs.count(yield_sign_id) == 1);
                  auto& yield_sign = _pb_map_data.pb_yield_signs[yield_sign_id];
                  YieldSignInternal yield_sign_internal;
                  StopLineInternal stop_line;
                  stop_line.id = create_stop_line_id();
                  stop_line.curve = yield_sign.stop_line();
                  road.stop_lines.push_back(stop_line);
                  yield_sign_internal.stop_line_ids.insert(stop_line.id);
                  yield_sign_internal.id = yield_sign.id().id();
                  yield_sign_internal.yield_sign = yield_sign;
                  road.yield_signs.push_back(yield_sign_internal);
                  overlap_with_lane.object_id = stop_line.id;
                  lane_internal.overlap_objects.push_back(overlap_with_lane);
                  overlap_with_lane.object_id = yield_sign_internal.id;
                  lane_internal.overlap_signals.push_back(overlap_with_lane);
              } else if (overlap_obj.has_crosswalk_overlap_info()) {
                  std::string crosswalk_id = overlap_obj.id().id();
                  assert(_pb_map_data.pb_crosswalks.count(crosswalk_id) == 1);
                  road.crosswalks.push_back(
                                  _pb_map_data.pb_crosswalks[crosswalk_id]);
                  overlap_with_lane.object_id = crosswalk_id;
                  lane_internal.overlap_objects.push_back(overlap_with_lane);
              } else if (overlap_obj.has_clear_area_overlap_info()) {
                  std::string clear_area_id = overlap_obj.id().id();
                  assert(_pb_map_data.pb_clear_areas.count(clear_area_id) == 1);
                  road.clear_areas.push_back(
                              _pb_map_data.pb_clear_areas[clear_area_id]);
                  overlap_with_lane.object_id = clear_area_id;
                  lane_internal.overlap_objects.push_back(overlap_with_lane);
              } else if (overlap_obj.has_speed_bump_overlap_info()) {
                  std::string speed_bump_id = overlap_obj.id().id();
                  assert(_pb_map_data.pb_speed_bumps.count(speed_bump_id) == 1);
                  road.speed_bumps.push_back(
                              _pb_map_data.pb_speed_bumps[speed_bump_id]);
                  overlap_with_lane.object_id = speed_bump_id;
                  lane_internal.overlap_objects.push_back(overlap_with_lane);
              } else if (overlap_obj.has_junction_overlap_info()) {
                  overlap_with_lane.object_id = overlap_obj.id().id();
                  lane_internal.overlap_junctions.push_back(
                                                          overlap_with_lane);
              } else if (overlap_obj.has_lane_overlap_info()) {
                  if (overlap_obj.id().id() == lane.id().id()) {
                      continue;
                  }
                  overlap_with_lane.object_id = overlap_obj.id().id();
                  lane_internal.overlap_lanes.push_back(overlap_with_lane);
              }
          }
      }
      road.lanes.push_back(lane_internal);
  }
  return roads;
}

std::vector<PbCrosswalk> ProtoOrganization::get_crosswalks() {
    std::vector<PbCrosswalk> crosswalks;
    for (int i = 0; i < _pb_map->crosswalk_size(); ++i) {
        auto& crosswalk = _pb_map->crosswalk(i);
        crosswalks.push_back(crosswalk);
    }
    return crosswalks;
}

std::vector<PbClearArea> ProtoOrganization::get_clear_areas() {
    std::vector<PbClearArea> clear_areas;
    for (int i = 0; i < _pb_map->clear_area_size(); ++i) {
        auto& clear_area = _pb_map->clear_area(i);
        clear_areas.push_back(clear_area);
    }
    return clear_areas;
}

std::vector<PbSpeedBump> ProtoOrganization::get_speed_bumps() {
    std::vector<PbSpeedBump> speed_bumps;
    for (int i = 0; i < _pb_map->speed_bump_size(); ++i) {
        auto& speed_bump = _pb_map->speed_bump(i);
        speed_bumps.push_back(speed_bump);
    }
    return speed_bumps;
}

std::vector<StopLineInternal> ProtoOrganization::get_stop_lines() {
    return _stop_lines;
}

std::vector<TrafficLightInternal> ProtoOrganization::get_traffic_lights() {
    std::vector<TrafficLightInternal> signals;
    for (int i = 0; i < _pb_map->signal_size(); ++i) {
        TrafficLightInternal signal_internal;
        auto& signal = _pb_map->signal(i);
        for (int j = 0; j < signal.stop_line_size(); ++j) {
            StopLineInternal stop_line;
            stop_line.id = create_stop_line_id();
            stop_line.curve = signal.stop_line(j);
            _stop_lines.push_back(stop_line);
            signal_internal.stop_line_ids.insert(stop_line.id);
        }
        signal_internal.id = signal.id().id();
        signal_internal.traffic_light = signal;
        signals.push_back(signal_internal);
    }
    return signals;
}

std::vector<StopSignInternal> ProtoOrganization::get_stop_signs() {
    std::vector<StopSignInternal> stop_signs;
    for (int i = 0; i < _pb_map->stop_sign_size(); ++i) {
        StopSignInternal stop_sign_internal;
        auto& stop_sign = _pb_map->stop_sign(i);
        // for (int j = 0; j < stop_sign.stop_line_size(); ++j) {
        StopLineInternal stop_line;
        stop_line.id = create_stop_line_id();
        stop_line.curve = stop_sign.stop_line();
        _stop_lines.push_back(stop_line);
        stop_sign_internal.stop_line_ids.insert(stop_line.id);
        //}
        stop_sign_internal.id = stop_sign.id().id();
        stop_sign_internal.stop_sign = stop_sign;
        stop_signs.push_back(stop_sign_internal);
    }
    return stop_signs;
}

std::vector<YieldSignInternal> ProtoOrganization::get_yield_signs() {
    std::vector<YieldSignInternal> yield_signs;
    for (int i = 0; i < _pb_map->yield_size(); ++i) {
        auto& yield_sign = _pb_map->yield(i);
        YieldSignInternal yield_sign_internal;
        // for (int j = 0; j < yield_sign.stop_line_size(); ++j) {
            StopLineInternal stop_line;
            stop_line.id = create_stop_line_id();
            stop_line.curve = yield_sign.stop_line();
            _stop_lines.push_back(stop_line);
            yield_sign_internal.stop_line_ids.insert(stop_line.id);
        // }
        yield_sign_internal.id = yield_sign.id().id();
        yield_sign_internal.yield_sign = yield_sign;
        yield_signs.push_back(yield_sign_internal);
    }
    return yield_signs;
}

std::vector<JunctionInternal> ProtoOrganization::get_junctions() {
    std::vector<JunctionInternal> junctions;
    // for (auto& road_in_junction_pair : _road_in_junctions) {
    //     std::cout << "road id:" << road_in_junction_pair.first << std::endl;
    //     for (auto& junction_id : road_in_junction_pair.second) {
    //         std::cout << "junction id:" << junction_id << std::endl;
    //     }
    // }

    for (int i = 0; i < _pb_map->junction_size(); ++i) {
        auto& junction = _pb_map->junction(i);

        JunctionInternal junction_internal;
        junction_internal.junction = junction;

        std::string junction_id = junction.id().id();
        for (auto& road_in_junction_pair : _road_in_junctions) {
            if (road_in_junction_pair.second.count(junction_id) > 0) {
                junction_internal.road_ids.insert(
                                            road_in_junction_pair.first);
            }
        }

        auto& overlaps = _pb_map_data.pb_overlaps;
        for (int j = 0; j < junction.overlap_id_size(); ++j) {
            const PbID& junction_overlap_id = junction.overlap_id(j);
            PbOverlap& junction_overlap = overlaps[junction_overlap_id.id()];
            for (int k = 0; k < junction_overlap.object_size(); ++k) {
                const PbObjectOverlapInfo& overlap_obj =
                                                     junction_overlap.object(k);
                if (overlap_obj.id().id() != junction.id().id()
                && !overlap_obj.has_lane_overlap_info()) {
                    OverlapWithJunction overlap_with_junction;
                    overlap_with_junction.object_id = overlap_obj.id().id();
                    junction_internal.overlap_with_junctions.push_back(
                                                        overlap_with_junction);
                }
            }
        }

        junctions.push_back(junction_internal);
    }

    return junctions;
}

void ProtoOrganization::get_road_elements(std::vector<Road>* roads,
                                        ProtoData* proto_data) {
    for (auto& road : *roads) {
        // lanes
        for (auto& lane_internal : road.lanes) {
            proto_data->pb_lanes[lane_internal.lane.id().id()]
                                                        = lane_internal.lane;
        }
        // crosswalks
        for (auto& crosswalk : road.crosswalks) {
            proto_data->pb_crosswalks[crosswalk.id().id()] = crosswalk;
        }
        // clear areas
        for (auto& clear_area : road.clear_areas) {
            proto_data->pb_clear_areas[clear_area.id().id()] = clear_area;
        }
        // speed_bump
        for (auto& speed_bump : road.speed_bumps) {
            proto_data->pb_speed_bumps[speed_bump.id().id()] = speed_bump;
        }
        // stop lines
        for (auto& stop_line_internal : road.stop_lines) {
            proto_data->pb_stop_lines[stop_line_internal.id]
                                                    = stop_line_internal;
        }

        // traffic_lights
        for (auto& traffic_light_internal : road.traffic_lights) {
            auto& traffic_light = traffic_light_internal.traffic_light;
            for (auto stop_line_id : traffic_light_internal.stop_line_ids) {
                assert(proto_data->pb_stop_lines.count(stop_line_id) > 0);
                auto& stop_line_curve =
                                proto_data->pb_stop_lines[stop_line_id].curve;
                (*traffic_light.add_stop_line()) = stop_line_curve;
            }
            proto_data->pb_signals[traffic_light.id().id()] = traffic_light;
        }
        // stop signs
        for (auto& stop_sign_internal : road.stop_signs) {
            auto& stop_sign = stop_sign_internal.stop_sign;
            for (auto stop_line_id : stop_sign_internal.stop_line_ids) {
                assert(proto_data->pb_stop_lines.count(stop_line_id) > 0);
                auto& stop_line_curve =
                                proto_data->pb_stop_lines[stop_line_id].curve;
                (*stop_sign.mutable_stop_line()) = stop_line_curve;
            }
            proto_data->pb_stop_signs[stop_sign.id().id()] = stop_sign;
        }
        // yield signs
        for (auto& yield_sign_internal : road.yield_signs) {
            auto& yield_sign = yield_sign_internal.yield_sign;
            for (auto stop_line_id : yield_sign_internal.stop_line_ids) {
                assert(proto_data->pb_stop_lines.count(stop_line_id) > 0);
                auto& stop_line_curve =
                                proto_data->pb_stop_lines[stop_line_id].curve;
                (*yield_sign.mutable_stop_line()) = stop_line_curve;
            }
            proto_data->pb_yield_signs[yield_sign.id().id()] = yield_sign;
        }
    }
}

void ProtoOrganization::get_junction_elements(
                        const std::vector<JunctionInternal>& junctions,
                        ProtoData* proto_data) {
    for (auto& junction_internal : junctions) {
        std::string junction_id = junction_internal.junction.id().id();
        proto_data->pb_junctions[junction_id] = junction_internal.junction;
    }
}

void ProtoOrganization::get_overlap_elements(
                        const std::vector<Road>& roads,
                        const std::vector<JunctionInternal>& junctions,
                        ProtoData* proto_data) {
  std::unordered_map<std::string, OverlapWithLane> lane_2_lane_overlaps;
  for (auto& road : roads) {
      for (auto& lane_internal : road.lanes) {
          for (auto& lane_2_lane_overlap : lane_internal.overlap_lanes) {
              lane_2_lane_overlaps[lane_2_lane_overlap.object_id] =
                                                      lane_2_lane_overlap;
          }
      }
  }
  std::unordered_set<std::string> close_table;
  // overlap
  for (auto& road : roads) {
      for (auto& lane_internal : road.lanes) {
          std::string lane_id = lane_internal.lane.id().id();
          for (auto& overlap_object : lane_internal.overlap_objects) {
              std::string object_id = overlap_object.object_id;
              if (proto_data->pb_crosswalks.count(object_id) <= 0
              && proto_data->pb_clear_areas.count(object_id) <= 0
              && proto_data->pb_speed_bumps.count(object_id) <= 0) {
                  continue;
              }
              PbOverlap overlap;
              std::string overlap_id = create_overlap_id();
              proto_data->pb_lanes[lane_id].add_overlap_id()
                        ->set_id(overlap_id);
              overlap.mutable_id()->set_id(overlap_id);
              PbObjectOverlapInfo* object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(lane_id);
              object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                      overlap_object.start_s);
              object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                      overlap_object.end_s);
              assert(proto_data->pb_lanes.count(lane_id) > 0);
              object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(object_id);
              if (proto_data->pb_crosswalks.count(object_id) > 0) {
                  proto_data->pb_crosswalks[object_id].add_overlap_id()
                        ->set_id(overlap_id);
                  object_overlap->mutable_crosswalk_overlap_info();
              } else if (proto_data->pb_clear_areas.count(object_id) > 0) {
                  object_overlap->mutable_clear_area_overlap_info();
                  proto_data->pb_clear_areas[object_id].add_overlap_id()
                        ->set_id(overlap_id);
              } else if (proto_data->pb_speed_bumps.count(object_id)) {
                  object_overlap->mutable_speed_bump_overlap_info();
                  proto_data->pb_speed_bumps[object_id].add_overlap_id()
                        ->set_id(overlap_id);
              } else {
                  assert(0);
              }
              proto_data->pb_overlaps[overlap_id] = overlap;
          }
          for (auto& overlap_signal : lane_internal.overlap_signals) {
              std::string object_id = overlap_signal.object_id;
              if (proto_data->pb_signals.count(object_id) <= 0
              && proto_data->pb_stop_signs.count(object_id) <= 0
              && proto_data->pb_yield_signs.count(object_id) <= 0) {
                  std::cout << "cannot find signal object_id:" << object_id
                      << std::endl;
                  continue;
              }
              PbOverlap overlap;
              std::string overlap_id = create_overlap_id();
              proto_data->pb_lanes[lane_id].add_overlap_id()
                        ->set_id(overlap_id);
              overlap.mutable_id()->set_id(overlap_id);
              PbObjectOverlapInfo* object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(lane_id);
              object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                      overlap_signal.start_s);
              object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                      overlap_signal.end_s);
              assert(proto_data->pb_lanes.count(lane_id) > 0);
              object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(object_id);
              if (proto_data->pb_signals.count(object_id) > 0) {
                  object_overlap->mutable_signal_overlap_info();
                  proto_data->pb_signals[object_id].add_overlap_id()->set_id(
                                                                  overlap_id);
              } else if (proto_data->pb_stop_signs.count(object_id) > 0) {
                  object_overlap->mutable_stop_sign_overlap_info();
                  proto_data->pb_stop_signs[object_id].add_overlap_id()->set_id(
                                                                  overlap_id);
              } else if (proto_data->pb_yield_signs.count(object_id) > 0) {
                  object_overlap->mutable_yield_sign_overlap_info();
                  proto_data->pb_yield_signs[object_id].add_overlap_id()
                                ->set_id(overlap_id);
              } else {
                  assert(0);
              }
              proto_data->pb_overlaps[overlap_id] = overlap;
          }
          for (auto& overlap_junction : lane_internal.overlap_junctions) {
              std::string object_id = overlap_junction.object_id;
              if (proto_data->pb_junctions.count(object_id) <= 0) {
                  std::cout << "cannot find junction object id:"
                    << object_id << std::endl;
                  continue;
              }
              PbOverlap overlap;
              std::string overlap_id = create_overlap_id();
              proto_data->pb_lanes[lane_id].add_overlap_id()
                        ->set_id(overlap_id);
              overlap.mutable_id()->set_id(overlap_id);
              PbObjectOverlapInfo* object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(lane_id);
              object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                  overlap_junction.start_s);
              object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                  overlap_junction.end_s);
              assert(proto_data->pb_lanes.count(lane_id) > 0);
              object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(object_id);
              if (proto_data->pb_junctions.count(object_id) > 0) {
                  object_overlap->mutable_junction_overlap_info();
                  proto_data->pb_junctions[object_id].add_overlap_id()->set_id(
                                                                  overlap_id);
              } else {
                  assert(0);
              }
              proto_data->pb_overlaps[overlap_id] = overlap;
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
              proto_data->pb_lanes[lane_id].add_overlap_id()->set_id(
                                                                  overlap_id);
              overlap.mutable_id()->set_id(overlap_id);
              PbObjectOverlapInfo* object_overlap = overlap.add_object();
              object_overlap->mutable_id()->set_id(lane_id);
              object_overlap->mutable_lane_overlap_info()->set_start_s(
                                                  overlap_lane.start_s);
              object_overlap->mutable_lane_overlap_info()->set_end_s(
                                                  overlap_lane.end_s);
              assert(proto_data->pb_lanes.count(object_id) > 0);
              object_overlap = overlap.add_object();
              if (proto_data->pb_lanes.count(object_id) > 0) {
                  assert(lane_2_lane_overlaps.count(object_id) == 1);
                  auto& lane_2_lane_overlap = lane_2_lane_overlaps[object_id];
                  object_overlap->mutable_lane_overlap_info()->set_start_s(
                      lane_2_lane_overlap.start_s);
                  object_overlap->mutable_lane_overlap_info()->set_end_s(
                      lane_2_lane_overlap.end_s);
                  object_overlap->mutable_lane_overlap_info()
                                                      ->set_is_merge(false);
                  object_overlap->mutable_lane_overlap_info()->
                                                  set_has_precedence(false);
                  proto_data->pb_lanes[object_id].add_overlap_id()->set_id(
                                                                  overlap_id);
              } else {
                  assert(0);
              }
              proto_data->pb_overlaps[overlap_id] = overlap;
          }
      }
  }

  for (auto& junction_internal : junctions) {
      std::string junction_id = junction_internal.junction.id().id();
      for (auto& overlap_junction :
                                  junction_internal.overlap_with_junctions) {
          // std::cout << "junction interanl id:" << junction_id << std::endl;
          PbOverlap overlap;
          std::string overlap_id = create_overlap_id();
          proto_data->pb_junctions[junction_id].add_overlap_id()->set_id(
                                                                  overlap_id);
          overlap.mutable_id()->set_id(overlap_id);
          PbObjectOverlapInfo* object_overlap = overlap.add_object();
          object_overlap->mutable_id()->set_id(junction_id);
          std::string object_id = overlap_junction.object_id;
          object_overlap = overlap.add_object();
          object_overlap->mutable_id()->set_id(object_id);
          if (proto_data->pb_crosswalks.count(object_id) > 0) {
              object_overlap->mutable_crosswalk_overlap_info();
              proto_data->pb_crosswalks[object_id].add_overlap_id()->set_id(
                                                              overlap_id);
          } else if (proto_data->pb_clear_areas.count(object_id) > 0) {
              object_overlap->mutable_clear_area_overlap_info();
              proto_data->pb_clear_areas[object_id].add_overlap_id()->set_id(
                                                              overlap_id);
          } else {
              assert(0);
          }
          proto_data->pb_overlaps[overlap_id] = overlap;
      }
  }
}

void ProtoOrganization::output_data(const ProtoData& proto_data,
                                            ::apollo::hdmap::Map* pb_map) {
    // int lane_size = 0;
    // int crosswalk_size = 0;
    // int clear_area_size = 0;
    // int speed_bump_size = 0;
    // int signal_size = 0;
    // int stop_sign_size = 0;
    // int yield_sign_size = 0;
    // int overlap_size = 0;
    // int junction_size = 0;

    for (auto& lane_pair : proto_data.pb_lanes) {
        // ++lane_size;
        *(pb_map->add_lane()) = lane_pair.second;
    }
    for (auto& crosswalk_pair : proto_data.pb_crosswalks) {
        // ++crosswalk_size;
        *(pb_map->add_crosswalk()) = crosswalk_pair.second;
    }
    for (auto& clear_area_pair : proto_data.pb_clear_areas) {
        // ++clear_area_size;
        *(pb_map->add_clear_area()) = clear_area_pair.second;
    }
    for (auto& speed_bump_pair : proto_data.pb_speed_bumps) {
        // ++speed_bump_size;
        *(pb_map->add_speed_bump()) = speed_bump_pair.second;
    }
    for (auto& signal_pair : proto_data.pb_signals) {
        // ++signal_size;
        *(pb_map->add_signal()) = signal_pair.second;
    }
    for (auto& stop_sign_pair : proto_data.pb_stop_signs) {
        // ++stop_sign_size;
        *(pb_map->add_stop_sign()) = stop_sign_pair.second;
    }
    for (auto& yield_sign_pair : proto_data.pb_yield_signs) {
        // ++yield_sign_size;
        *(pb_map->add_yield()) = yield_sign_pair.second;
    }

    for (auto& junction_pair : proto_data.pb_junctions) {
        // ++junction_size;
        *(pb_map->add_junction()) = junction_pair.second;
    }

    for (auto& overlap_pair : proto_data.pb_overlaps) {
        // ++overlap_size;
        *(pb_map->add_overlap()) = overlap_pair.second;
    }

    // std::cout << "total: lane-" << lane_size
    //     << ";crosswalk-" << crosswalk_size
    //     << ";clear area-" << clear_area_size
    //     << ";speed bump-" << speed_bump_size
    //     << ";signal-" << signal_size
    //     << ";stop sign-" << stop_sign_size
    //     << ";yield sign-" << yield_sign_size
    //     << ";junction-" << junction_size
    //     << ";overlap-" << overlap_size
    //     << std::endl;
}

bool ProtoOrganization::point_in_lane_boundary(const PbPoint3D& pt,
                                            const PbLane& lane) {
    auto& left_boundary_curve = lane.left_boundary().curve();
    auto& right_boundary_curve = lane.right_boundary().curve();
    std::vector<apollo::common::math::Vec2d> lane_boundary;
    std::vector<apollo::common::math::Vec2d> left_boundary;
    std::vector<apollo::common::math::Vec2d> right_boundary;
    points_from_curve(left_boundary_curve, &left_boundary);
    points_from_curve(right_boundary_curve, &right_boundary);
    lane_boundary.insert(lane_boundary.end(), left_boundary.begin(),
                        left_boundary.end());
    lane_boundary.insert(lane_boundary.end(), right_boundary.rbegin(),
                        right_boundary.rend());
    apollo::common::math::Polygon2d boundary_polygon(lane_boundary);
    apollo::common::math::Vec2d target_pt(pt.x(), pt.y());
    if (boundary_polygon.IsPointIn(target_pt)) {
        return true;
    }

    return false;
}

bool ProtoOrganization::stop_line_in_lane_boundary(const PbCurve& stop_line,
                                                        const PbLane& lane) {
    std::vector<PbPoint3D> stop_line_curve;
    for (const auto &curve : stop_line.segment()) {
        if (curve.has_line_segment()) {
            for (const auto &point : curve.line_segment().point()) {
                stop_line_curve.push_back(point);
            }
        } else {
           assert(0);
        }
    }

    return point_in_lane_boundary(stop_line_curve[1], lane);
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
