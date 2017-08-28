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

#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace hdmap {

int HDMap::LoadMapFromFile(const std::string& map_filename) {
  return _impl.LoadMapFromFile(map_filename);
}

LaneInfoConstPtr HDMap::GetLaneById(const Id& id) const {
  return _impl.GetLaneById(id);
}

JunctionInfoConstPtr HDMap::GetJunctionById(const Id& id) const {
  return _impl.GetJunctionById(id);
}

SignalInfoConstPtr HDMap::GetSignalById(const Id& id) const {
  return _impl.GetSignalById(id);
}

CrosswalkInfoConstPtr HDMap::GetCrosswalkById(const Id& id) const {
  return _impl.GetCrosswalkById(id);
}

StopSignInfoConstPtr HDMap::GetStopSignById(const Id& id) const {
  return _impl.GetStopSignById(id);
}

YieldSignInfoConstPtr HDMap::GetYieldSignById(const Id& id) const {
  return _impl.GetYieldSignById(id);
}

OverlapInfoConstPtr HDMap::GetOverlapById(const Id& id) const {
  return _impl.GetOverlapById(id);
}

RoadInfoConstPtr HDMap::GetRoadById(const Id& id) const {
  return _impl.GetRoadById(id);
}

int HDMap::GetLanes(const apollo::common::PointENU& point, double distance,
                    std::vector<LaneInfoConstPtr>* lanes) const {
  return _impl.GetLanes(point, distance, lanes);
}

int HDMap::GetJunctions(const apollo::common::PointENU& point, double distance,
                        std::vector<JunctionInfoConstPtr>* junctions) const {
  return _impl.GetJunctions(point, distance, junctions);
}

int HDMap::GetSignals(const apollo::common::PointENU& point, double distance,
                      std::vector<SignalInfoConstPtr>* signals) const {
  return _impl.GetSignals(point, distance, signals);
}

int HDMap::GetCrosswalks(const apollo::common::PointENU& point, double distance,
                         std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  return _impl.GetCrosswalks(point, distance, crosswalks);
}

int HDMap::GetStopSigns(const apollo::common::PointENU& point, double distance,
                        std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return _impl.GetStopSigns(point, distance, stop_signs);
}

int HDMap::GetYieldSigns(
    const apollo::common::PointENU& point, double distance,
    std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  return _impl.GetYieldSigns(point, distance, yield_signs);
}

int HDMap::GetRoads(const apollo::common::PointENU& point, double distance,
                    std::vector<RoadInfoConstPtr>* roads) const {
  return _impl.GetRoads(point, distance, roads);
}

int HDMap::GetNearestLane(const ::apollo::common::PointENU& point,
                          LaneInfoConstPtr* nearest_lane,
                          double* nearest_s, double* nearest_l) const {
  return _impl.GetNearestLane(point, nearest_lane, nearest_s, nearest_l);
}

int HDMap::GetNearestLaneWithHeading(const apollo::common::PointENU& point,
                                     const double distance,
                                     const double central_heading,
                                     const double max_heading_difference,
                                     LaneInfoConstPtr* nearest_lane,
                                     double* nearest_s,
                                     double* nearest_l) const {
  return _impl.GetNearestLaneWithHeading(
      point, distance, central_heading, max_heading_difference, nearest_lane,
      nearest_s, nearest_l);
}

int HDMap::GetLanesWithHeading(const apollo::common::PointENU& point,
                               const double distance,
                               const double central_heading,
                               const double max_heading_difference,
                               std::vector<LaneInfoConstPtr>* lanes) const {
  return _impl.GetLanesWithHeading(
      point, distance, central_heading, max_heading_difference, lanes);
}

int HDMap::GetRoadBoundaries(
    const apollo::common::PointENU& point, double radius,
    std::vector<RoadROIBoundaryPtr>* road_boundaries,
    std::vector<JunctionBoundaryPtr>* junctions) const {
  return _impl.GetRoadBoundaries(point, radius, road_boundaries, junctions);
}

}  // namespace hdmap
}  // namespace apollo
