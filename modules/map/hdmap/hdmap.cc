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
  AINFO << "Loading HDMap: " << map_filename << " ...";
  return impl_.LoadMapFromFile(map_filename);
}

int HDMap::LoadMapFromProto(const Map& map_proto) {
  ADEBUG << "Loading HDMap with header: "
         << map_proto.header().ShortDebugString();
  return impl_.LoadMapFromProto(map_proto);
}

LaneInfoConstPtr HDMap::GetLaneById(const Id& id) const {
  return impl_.GetLaneById(id);
}

JunctionInfoConstPtr HDMap::GetJunctionById(const Id& id) const {
  return impl_.GetJunctionById(id);
}

SignalInfoConstPtr HDMap::GetSignalById(const Id& id) const {
  return impl_.GetSignalById(id);
}

CrosswalkInfoConstPtr HDMap::GetCrosswalkById(const Id& id) const {
  return impl_.GetCrosswalkById(id);
}

StopSignInfoConstPtr HDMap::GetStopSignById(const Id& id) const {
  return impl_.GetStopSignById(id);
}

YieldSignInfoConstPtr HDMap::GetYieldSignById(const Id& id) const {
  return impl_.GetYieldSignById(id);
}

ClearAreaInfoConstPtr HDMap::GetClearAreaById(const Id& id) const {
  return impl_.GetClearAreaById(id);
}

SpeedBumpInfoConstPtr HDMap::GetSpeedBumpById(const Id& id) const {
  return impl_.GetSpeedBumpById(id);
}

OverlapInfoConstPtr HDMap::GetOverlapById(const Id& id) const {
  return impl_.GetOverlapById(id);
}

RoadInfoConstPtr HDMap::GetRoadById(const Id& id) const {
  return impl_.GetRoadById(id);
}

int HDMap::GetLanes(const apollo::common::PointENU& point, double distance,
                    std::vector<LaneInfoConstPtr>* lanes) const {
  return impl_.GetLanes(point, distance, lanes);
}

int HDMap::GetJunctions(const apollo::common::PointENU& point, double distance,
                        std::vector<JunctionInfoConstPtr>* junctions) const {
  return impl_.GetJunctions(point, distance, junctions);
}

int HDMap::GetSignals(const apollo::common::PointENU& point, double distance,
                      std::vector<SignalInfoConstPtr>* signals) const {
  return impl_.GetSignals(point, distance, signals);
}

int HDMap::GetCrosswalks(const apollo::common::PointENU& point, double distance,
                         std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  return impl_.GetCrosswalks(point, distance, crosswalks);
}

int HDMap::GetStopSigns(const apollo::common::PointENU& point, double distance,
                        std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return impl_.GetStopSigns(point, distance, stop_signs);
}

int HDMap::GetYieldSigns(
    const apollo::common::PointENU& point, double distance,
    std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  return impl_.GetYieldSigns(point, distance, yield_signs);
}

int HDMap::GetClearAreas(
    const apollo::common::PointENU& point, double distance,
    std::vector<ClearAreaInfoConstPtr>* clear_areas) const {
  return impl_.GetClearAreas(point, distance, clear_areas);
}

int HDMap::GetSpeedBumps(
    const apollo::common::PointENU& point, double distance,
    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const {
  return impl_.GetSpeedBumps(point, distance, speed_bumps);
}

int HDMap::GetRoads(const apollo::common::PointENU& point, double distance,
                    std::vector<RoadInfoConstPtr>* roads) const {
  return impl_.GetRoads(point, distance, roads);
}

int HDMap::GetNearestLane(const common::PointENU& point,
                          LaneInfoConstPtr* nearest_lane, double* nearest_s,
                          double* nearest_l) const {
  return impl_.GetNearestLane(point, nearest_lane, nearest_s, nearest_l);
}

int HDMap::GetNearestLaneWithHeading(const apollo::common::PointENU& point,
                                     const double distance,
                                     const double central_heading,
                                     const double max_heading_difference,
                                     LaneInfoConstPtr* nearest_lane,
                                     double* nearest_s,
                                     double* nearest_l) const {
  return impl_.GetNearestLaneWithHeading(point, distance, central_heading,
                                         max_heading_difference, nearest_lane,
                                         nearest_s, nearest_l);
}

int HDMap::GetLanesWithHeading(const apollo::common::PointENU& point,
                               const double distance,
                               const double central_heading,
                               const double max_heading_difference,
                               std::vector<LaneInfoConstPtr>* lanes) const {
  return impl_.GetLanesWithHeading(point, distance, central_heading,
                                   max_heading_difference, lanes);
}

int HDMap::GetRoadBoundaries(
    const apollo::common::PointENU& point, double radius,
    std::vector<RoadROIBoundaryPtr>* road_boundaries,
    std::vector<JunctionBoundaryPtr>* junctions) const {
  return impl_.GetRoadBoundaries(point, radius, road_boundaries, junctions);
}

int HDMap::GetForwardNearestSignalsOnLane(
    const apollo::common::PointENU& point, const double distance,
    std::vector<SignalInfoConstPtr>* signals) const {
  return impl_.GetForwardNearestSignalsOnLane(point, distance, signals);
}

int HDMap::GetStopSignAssociatedStopSigns(
    const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return impl_.GetStopSignAssociatedStopSigns(id, stop_signs);
}

int HDMap::GetStopSignAssociatedLanes(
    const Id& id, std::vector<LaneInfoConstPtr>* lanes) const {
  return impl_.GetStopSignAssociatedLanes(id, lanes);
}

}  // namespace hdmap
}  // namespace apollo
