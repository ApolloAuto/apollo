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

#include "modules/map/hdmap/hdmap.h"

namespace apollo {
namespace hdmap {
int HDMap::load_map_from_file(const std::string& map_filename) {
  return _impl.load_map_from_file(map_filename);
}

LaneInfoConstPtr HDMap::get_lane_by_id(const apollo::hdmap::Id& id) const {
    return _impl.get_lane_by_id(id);
}

JunctionInfoConstPtr HDMap::get_junction_by_id(
                                            const apollo::hdmap::Id& id) const {
    return _impl.get_junction_by_id(id);
}

SignalInfoConstPtr HDMap::get_signal_by_id(const apollo::hdmap::Id& id) const {
    return _impl.get_signal_by_id(id);
}

CrosswalkInfoConstPtr HDMap::get_crosswalk_by_id(
                                            const apollo::hdmap::Id& id) const {
    return _impl.get_crosswalk_by_id(id);
}

StopSignInfoConstPtr HDMap::get_stop_sign_by_id(
                                            const apollo::hdmap::Id& id) const {
    return _impl.get_stop_sign_by_id(id);
}

YieldSignInfoConstPtr HDMap::get_yield_sign_by_id(
                                            const apollo::hdmap::Id& id) const {
    return _impl.get_yield_sign_by_id(id);
}

OverlapInfoConstPtr HDMap::get_overlap_by_id(
                                            const apollo::hdmap::Id& id) const {
    return _impl.get_overlap_by_id(id);
}

int HDMap::get_lanes(const apollo::hdmap::Point& point,
                     double distance,
                     std::vector<LaneInfoConstPtr>* lanes) const {
    return _impl.get_lanes(point, distance, lanes);
}

int HDMap::get_junctions(const apollo::hdmap::Point& point,
                         double distance,
                         std::vector<JunctionInfoConstPtr>* junctions) const {
    return _impl.get_junctions(point, distance, junctions);
}

int HDMap::get_signals(const apollo::hdmap::Point& point,
                       double distance,
                       std::vector<SignalInfoConstPtr>* signals) const {
    return _impl.get_signals(point, distance, signals);
}

int HDMap::get_crosswalks(const apollo::hdmap::Point& point,
                        double distance,
                        std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
    return _impl.get_crosswalks(point, distance, crosswalks);
}

int HDMap::get_stop_signs(const apollo::hdmap::Point& point,
                          double distance,
                          std::vector<StopSignInfoConstPtr>* stop_signs) const {
    return _impl.get_stop_signs(point, distance, stop_signs);
}

int HDMap::get_yield_signs(const apollo::hdmap::Point& point,
                        double distance,
                        std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  return _impl.get_yield_signs(point, distance, yield_signs);
}

int HDMap::get_nearest_lane(const ::apollo::hdmap::Point& point,
                LaneInfoConstPtr* nearest_lane,
                double* nearest_s,
                double* nearest_l) const {
  return _impl.get_nearest_lane(point, nearest_lane, nearest_s, nearest_l);
}

int HDMap::get_nearest_lane_with_heading(const apollo::hdmap::Point& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s,
                                double* nearest_l) const {
  return _impl.get_nearest_lane_with_heading(point, distance, central_heading,
                        max_heading_difference, nearest_lane,
                        nearest_s, nearest_l);
}

int HDMap::get_lanes_with_heading(const apollo::hdmap::Point& point,
                            const double distance,
                            const double central_heading,
                            const double max_heading_difference,
                            std::vector<LaneInfoConstPtr>* lanes) const {
  return _impl.get_lanes_with_heading(point, distance, central_heading,
                            max_heading_difference, lanes);
}

int HDMap::get_road_boundaries(const apollo::hdmap::Point& point,
                          double radius,
                          std::vector<RoadROIBoundaryPtr>* road_boundaries,
                          std::vector<JunctionInfoConstPtr>* junctions) const {
  return -1;
}

}  // namespace hdmap
}  // namespace apollo
