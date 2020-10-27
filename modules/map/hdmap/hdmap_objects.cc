/* Copyright 2020 The Apollo Authors. All Rights Reserved.

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

#include "modules/map/hdmap/hdmap_objects.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace apollo {
namespace hdmap {
namespace objects {

int StopSigns::GetAssociatedLanes(const Id& id,
                                  std::vector<LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);
  const auto& stop_sign = this->GetById(id);
  if (stop_sign == nullptr) {
    return -1;
  }

  std::vector<Id> associate_stop_sign_ids;
  const auto junction_ids = stop_sign->OverlapJunctionIds();
  for (const auto& junction_id : junction_ids) {
    const auto& junction = junctions_.GetById(junction_id);
    if (junction == nullptr) {
      continue;
    }
    const auto stop_sign_ids = junction->OverlapStopSignIds();
    std::copy(stop_sign_ids.begin(), stop_sign_ids.end(),
              std::back_inserter(associate_stop_sign_ids));
  }
  std::vector<Id> associate_lane_ids;
  for (const auto& stop_sign_id : associate_stop_sign_ids) {
    if (stop_sign_id.id() != id.id()) {
      const auto& stop_sign = this->GetById(stop_sign_id);
      if (stop_sign != nullptr) {
        const auto lane_ids = stop_sign->OverlapLaneIds();
        std::copy(lane_ids.begin(), lane_ids.end(),
                  std::back_inserter(associate_lane_ids));
      }
    }
  }

  for (const auto lane_id : associate_lane_ids) {
    const auto lane = lanes_.GetById(lane_id);
    if (lane != nullptr) {
      lanes->push_back(lane);
    }
  }

  return 0;
}

int StopSigns::GetAssociatedSigns(
    const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const {
  CHECK_NOTNULL(stop_signs);

  const auto& stop_sign = this->GetById(id);
  if (stop_sign == nullptr) {
    return -1;
  }

  std::vector<Id> associate_stop_sign_ids;
  const auto junction_ids = stop_sign->OverlapJunctionIds();
  for (const auto& junction_id : junction_ids) {
    const auto& junction = junctions_.GetById(junction_id);
    if (junction == nullptr) {
      continue;
    }
    const auto stop_sign_ids = junction->OverlapStopSignIds();
    std::copy(stop_sign_ids.begin(), stop_sign_ids.end(),
              std::back_inserter(associate_stop_sign_ids));
  }

  std::vector<Id> associate_lane_ids;
  for (const auto& stop_sign_id : associate_stop_sign_ids) {
    if (stop_sign_id.id() == id.id()) {
      // exclude current stop sign
      continue;
    }
    const auto& stop_sign = this->GetById(stop_sign_id);
    if (stop_sign == nullptr) {
      continue;
    }
    stop_signs->push_back(stop_sign);
  }

  return 0;
}

// default lanes search radius in GetForwardNearestSignalsOnLane
constexpr double kLanesSearchRange = 10.0;
// backward search distance in GetForwardNearestSignalsOnLane
constexpr int kBackwardDistance = 4;

int Lanes::GetRoadIds(const Vec2d& point, double distance,
                      RoadIds* road_ids) const {
  std::vector<LaneInfoConstPtr> lanes;
  if (road_ids == nullptr || this->GetItems(point, distance, &lanes) != 0) {
    return -1;
  }

  road_ids->reserve(lanes.size());
  for (const auto& lane : lanes) {
    auto& id = lane->road_id().id();
    if (!id.empty()) {
      road_ids->insert(id);
    }
  }

  return 0;
}

int Lanes::GetNearest(const PointENU& point, LaneInfoConstPtr* nearest_lane,
                      double* nearest_s, double* nearest_l) const {
  return GetNearest({point.x(), point.y()}, nearest_lane, nearest_s, nearest_l);
}

int Lanes::GetNearest(const Vec2d& point, LaneInfoConstPtr* nearest_lane,
                      double* nearest_s, double* nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);
  const auto* segment_object = kdtree_->GetNearestObject(point);
  if (segment_object == nullptr) {
    return -1;
  }

  const Id& lane_id = segment_object->object()->id();
  *nearest_lane = GetById(lane_id);
  ACHECK(*nearest_lane);
  const int id = segment_object->id();
  const auto& segment = (*nearest_lane)->segments()[id];
  Vec2d nearest_pt;
  segment.DistanceTo(point, &nearest_pt);
  *nearest_s = (*nearest_lane)->accumulate_s()[id] +
               nearest_pt.DistanceTo(segment.start());
  *nearest_l = segment.unit_direction().CrossProd(point - segment.start());

  return 0;
}

int Lanes::GetItemsWithHeading(const PointENU& point, const double distance,
                               const double central_heading,
                               const double max_heading_difference,
                               std::vector<LaneInfoConstPtr>* lanes) const {
  return GetItemsWithHeading({point.x(), point.y()}, distance, central_heading,
                             max_heading_difference, lanes);
}

int Lanes::GetItemsWithHeading(const Vec2d& point, const double distance,
                               const double central_heading,
                               const double max_heading_difference,
                               std::vector<LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = this->GetItems(point, distance, &all_lanes);
  if (status < 0 || all_lanes.empty()) {
    return -1;
  }

  lanes->clear();
  for (auto& lane : all_lanes) {
    Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
    if (dis <= distance) {
      double heading_diff =
          fabs(lane->headings()[s_offset_index] - central_heading);
      if (fabs(apollo::common::math::NormalizeAngle(heading_diff)) <=
          max_heading_difference) {
        lanes->push_back(lane);
      }
    }
  }

  return 0;
}

int Lanes::GetNearestWithHeading(const Vec2d& point, const double distance,
                                 const double central_heading,
                                 const double max_heading_difference,
                                 LaneInfoConstPtr* nearest_lane,
                                 double* nearest_s, double* nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);

  std::vector<LaneInfoConstPtr> lanes;
  if (this->GetItemsWithHeading(point, distance, central_heading,
                                max_heading_difference, &lanes) != 0) {
    return -1;
  }

  double s = 0;
  size_t s_index = 0;
  Vec2d map_point;
  double min_distance = distance;
  for (const auto& lane : lanes) {
    double s_offset = 0.0;
    int s_offset_index = 0;
    double distance =
        lane->DistanceTo(point, &map_point, &s_offset, &s_offset_index);
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
  int segment_index = static_cast<int>(
      std::min(s_index, (*nearest_lane)->segments().size() - 1));
  const auto& segment_2d = (*nearest_lane)->segments()[segment_index];
  *nearest_l =
      segment_2d.unit_direction().CrossProd(point - segment_2d.start());

  return 0;
}

int Lanes::GetForwardNearestSignalsOnLane(
    const apollo::common::PointENU& point, const double distance,
    std::vector<SignalInfoConstPtr>* signals) const {
  CHECK_NOTNULL(signals);

  signals->clear();
  LaneInfoConstPtr lane_ptr = nullptr;

  std::vector<LaneInfoConstPtr> temp_surrounding_lanes;
  std::vector<LaneInfoConstPtr> surrounding_lanes;
  int s_index = 0;
  apollo::common::math::Vec2d car_point;
  car_point.set_x(point.x());
  car_point.set_y(point.y());
  apollo::common::math::Vec2d map_point;
  if (this->GetItems(point, kLanesSearchRange, &temp_surrounding_lanes) == -1) {
    AINFO << "Can not find lanes around car.";
    return -1;
  }
  for (const auto& surround_lane : temp_surrounding_lanes) {
    if (surround_lane->IsOnLane(car_point)) {
      surrounding_lanes.push_back(surround_lane);
    }
  }
  if (surrounding_lanes.empty()) {
    AINFO << "Car is not on lane.";
    return -1;
  }

  double nearest_s = 0.0;
  double nearest_l = 0.0;
  for (const auto& lane : surrounding_lanes) {
    if (!lane->signals().empty()) {
      lane_ptr = lane;
      nearest_l =
          lane_ptr->DistanceTo(car_point, &map_point, &nearest_s, &s_index);
      break;
    }
  }
  if (lane_ptr == nullptr) {
    this->GetNearest(point, &lane_ptr, &nearest_s, &nearest_l);
    if (lane_ptr == nullptr) {
      return -1;
    }
  }

  double unused_distance = distance + kBackwardDistance;
  double back_distance = kBackwardDistance;
  double s = nearest_s;
  while (s < back_distance) {
    for (const auto& predecessor_lane_id :
         lane_ptr->inner_object().predecessor_id()) {
      lane_ptr = this->GetById(predecessor_lane_id);
      if (lane_ptr->inner_object().turn() == apollo::hdmap::Lane::NO_TURN) {
        break;
      }
    }
    back_distance = back_distance - s;
    s = lane_ptr->total_length();
  }
  double s_start = s - back_distance;
  while (lane_ptr != nullptr) {
    double signal_min_dist = std::numeric_limits<double>::infinity();
    std::vector<SignalInfoConstPtr> min_dist_signal_ptr;
    for (const auto& overlap_id : lane_ptr->inner_object().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = overlaps_.GetById(overlap_id);
      double lane_overlap_offset_s = 0.0;
      SignalInfoConstPtr signal_ptr = nullptr;
      auto& overlap = overlap_ptr->inner_object();
      for (int i = 0; i != overlap.object_size(); ++i) {
        auto& current_object = overlap.object(i);
        if (current_object.id().id() == lane_ptr->id().id()) {
          lane_overlap_offset_s =
              current_object.lane_overlap_info().start_s() - s_start;
          continue;
        }
        signal_ptr = signals_.GetById(current_object.id());
        if (signal_ptr == nullptr || lane_overlap_offset_s < 0.0) {
          break;
        }
        if (lane_overlap_offset_s < signal_min_dist) {
          signal_min_dist = lane_overlap_offset_s;
          min_dist_signal_ptr.clear();
          min_dist_signal_ptr.push_back(signal_ptr);
        } else if (lane_overlap_offset_s < (signal_min_dist + 0.1) &&
                   lane_overlap_offset_s > (signal_min_dist - 0.1)) {
          min_dist_signal_ptr.push_back(signal_ptr);
        }
      }
    }
    if (!min_dist_signal_ptr.empty() && unused_distance >= signal_min_dist) {
      *signals = min_dist_signal_ptr;
      break;
    }
    unused_distance = unused_distance - (lane_ptr->total_length() - s_start);
    if (unused_distance <= 0) {
      break;
    }
    LaneInfoConstPtr tmp_lane_ptr = nullptr;
    for (const auto& successor_lane_id :
         lane_ptr->inner_object().successor_id()) {
      tmp_lane_ptr = this->GetById(successor_lane_id);
      if (tmp_lane_ptr->inner_object().turn() == apollo::hdmap::Lane::NO_TURN) {
        break;
      }
    }
    lane_ptr = tmp_lane_ptr;
    s_start = 0;
  }
  return 0;
}

Roads::Roads(Lanes* lanes, const Junctions& junctions, const Overlaps& overlaps,
             const ParkingSpaces& parking_spaces)
    : lanes_(lanes),
      junctions_(junctions),
      overlaps_(overlaps),
      parking_spaces_(parking_spaces) {}

int Roads::GetItems(const PointENU& point, double distance,
                    std::vector<RoadInfoConstPtr>* roads) const {
  return this->GetItems({point.x(), point.y()}, distance, roads);
}

int Roads::GetItems(const Vec2d& point, double distance,
                    std::vector<RoadInfoConstPtr>* roads) const {
  Ids road_ids;
  auto result = lanes_->GetRoadIds(point, distance, &road_ids);
  if (result != 0) {
    return result;
  }

  roads->reserve(road_ids.size());
  for (auto& road_id : road_ids) {
    RoadInfoConstPtr road = GetById(CreateHDMapId(road_id));
    CHECK_NOTNULL(road);
    roads->push_back(road);
  }

  return 0;
}

int Roads::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadROIBoundaryPtr>* road_boundaries,
    std::vector<JunctionBoundaryPtr>* junctions) const {
  CHECK_NOTNULL(road_boundaries);
  CHECK_NOTNULL(junctions);

  road_boundaries->clear();
  junctions->clear();

  std::vector<LaneInfoConstPtr> lanes;
  if (lanes_->GetItems(point, radius, &lanes) != 0 || lanes.empty()) {
    return -1;
  }

  std::unordered_set<std::string> junction_id_set;
  junction_id_set.reserve(lanes.size() / 2);

  road_boundaries->reserve(lanes.size() / 2);

  std::unordered_set<std::string> road_section_id_set;
  road_section_id_set.reserve(lanes.size());

  for (const auto& lane : lanes) {
    const auto& road_id = lane->road_id();
    const auto& section_id = lane->section_id();
    std::string unique_id = road_id.id() + section_id.id();
    if (road_section_id_set.count(unique_id) > 0) {
      continue;
    }
    road_section_id_set.emplace(std::move(unique_id));
    const auto road_ptr = GetById(road_id);
    if (road_ptr == nullptr) {
      AERROR << "road id [" << road_id.id() << "] is not found.";
      continue;
    }
    if (road_ptr->has_junction_id()) {
      const Id junction_id = road_ptr->junction_id();
      const auto& junction_raw_id = junction_id.id();
      if (junction_id_set.count(junction_raw_id) > 0) {
        continue;
      }
      junction_id_set.insert(junction_raw_id);
      JunctionBoundaryPtr junction_boundary_ptr(new JunctionBoundary());
      junction_boundary_ptr->junction_info = junctions_.GetById(junction_id);
      if (junction_boundary_ptr->junction_info == nullptr) {
        AERROR << "junction id [" << junction_raw_id << "] is not found.";
        continue;
      }
      junctions->push_back(junction_boundary_ptr);
    } else {
      RoadROIBoundaryPtr road_boundary_ptr(new RoadROIBoundary());
      road_boundary_ptr->mutable_id()->CopyFrom(road_ptr->id());
      const auto& section_raw_id = section_id.id();
      for (const auto& section : road_ptr->sections()) {
        if (section.id().id() == section_raw_id) {
          road_boundary_ptr->add_road_boundaries()->CopyFrom(
              section.boundary());
        }
      }
      road_boundaries->push_back(road_boundary_ptr);
    }
  }

  return 0;
}

void Roads::GetBoundaries(std::vector<RoadRoiPtr>* roads_roi,
                          const RoadInfoConstPtr& road_ptr) {
  const std::vector<apollo::hdmap::RoadBoundary>& temp_roads_roi =
      road_ptr->GetBoundaries();
  if (!temp_roads_roi.empty()) {
    RoadRoiPtr road_boundary_ptr(new RoadRoi());
    road_boundary_ptr->id = road_ptr->id();
    for (const auto& temp_road_boundary : temp_roads_roi) {
      const BoundaryPolygon& boundary_polygon =
          temp_road_boundary.outer_polygon();

      for (const auto& edge : boundary_polygon.edge()) {
        LineBoundary* line_boundary;
        switch (edge.type()) {
          case apollo::hdmap::BoundaryEdge::LEFT_BOUNDARY:
            line_boundary = &road_boundary_ptr->left_boundary;
            break;
          case apollo::hdmap::BoundaryEdge::RIGHT_BOUNDARY:
            line_boundary = &road_boundary_ptr->right_boundary;
            break;
          default:
            continue;
        }

        for (const auto& s : edge.curve().segment()) {
          for (const auto& p : s.line_segment().point()) {
            line_boundary->line_points.push_back(p);
          }
        }
      }

      if (temp_road_boundary.hole_size() != 0) {
        for (const auto& hole : temp_road_boundary.hole()) {
          PolygonBoundary hole_boundary;
          for (const auto& edge : hole.edge()) {
            if (edge.type() == apollo::hdmap::BoundaryEdge::NORMAL) {
              for (const auto& s : edge.curve().segment()) {
                for (const auto& p : s.line_segment().point()) {
                  hole_boundary.polygon_points.push_back(p);
                }
              }
            }
          }

          road_boundary_ptr->holes_boundary.emplace_back(
              std::move(hole_boundary));
        }
      }
    }

    roads_roi->push_back(road_boundary_ptr);
  }
}

int Roads::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  if (road_boundaries == nullptr || junctions == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  road_boundaries->clear();
  junctions->clear();
  std::unordered_set<std::string> junction_id_set;
  std::vector<RoadInfoConstPtr> roads;
  if (GetItems(point, radius, &roads) != 0) {
    AERROR << "can not get roads in the range.";
    return -1;
  }

  road_boundaries->reserve(roads.size() / 2);
  junctions->reserve(roads.size() / 2);
  junction_id_set.reserve(roads.size() / 2);
  for (const auto& road_ptr : roads) {
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          junctions_.GetById(road_ptr->junction_id());
      const auto& junction_raw_id = junction_ptr->id().id();
      if (junction_id_set.find(junction_raw_id) == junction_id_set.end()) {
        junctions->push_back(junction_ptr);
        junction_id_set.insert(junction_raw_id);
      }
    } else {
      GetBoundaries(road_boundaries, road_ptr);
    }
  }
  return 0;
}

int Roads::GetRoi(const apollo::common::PointENU& point, double radius,
                  std::vector<RoadRoiPtr>* roads_roi,
                  std::vector<PolygonRoiPtr>* polygons_roi) {
  if (roads_roi == nullptr || polygons_roi == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  roads_roi->clear();
  polygons_roi->clear();
  std::unordered_set<std::string> polygon_id_set;
  std::vector<RoadInfoConstPtr> roads;
  std::vector<LaneInfoConstPtr> lanes;
  if (this->GetItems(point, radius, &roads) != 0) {
    AERROR << "can not get roads in the range.";
    return -1;
  }
  if (lanes_->GetItems(point, radius, &lanes) != 0) {
    AERROR << "can not get lanes in the range.";
    return -1;
  }

  roads_roi->reserve(roads.size() / 2);
  polygons_roi->reserve(roads.size() / 2);
  polygon_id_set.reserve(roads.size() / 2);
  for (const auto& road_ptr : roads) {
    // get junction polygon
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          junctions_.GetById(road_ptr->junction_id());
      const auto& junction_raw_id = junction_ptr->id().id();
      if (polygon_id_set.find(junction_raw_id) == polygon_id_set.end()) {
        PolygonRoiPtr polygon_roi_ptr(new PolygonRoi());
        polygon_roi_ptr->polygon = junction_ptr->polygon();
        polygon_roi_ptr->attribute.type = PolygonType::JUNCTION_POLYGON;
        polygon_roi_ptr->attribute.id = junction_ptr->id();
        polygons_roi->push_back(polygon_roi_ptr);
        polygon_id_set.insert(junction_raw_id);
      }
    } else {
      GetBoundaries(roads_roi, road_ptr);
    }
  }

  for (const auto& lane_ptr : lanes) {
    // get parking space polygon
    const auto& lane_raw_id = lane_ptr->id().id();
    for (const auto& overlap_id : lane_ptr->inner_object().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = overlaps_.GetById(overlap_id);
      const auto& overlap = overlap_ptr->inner_object();
      for (int i = 0; i != overlap.object_size(); ++i) {
        const auto& overlap_object = overlap.object(i);
        const auto& overlap_object_id = overlap_object.id();
        if (overlap_object_id.id() != lane_raw_id) {
          ParkingSpaceInfoConstPtr parkingspace_ptr =
              parking_spaces_.GetById(overlap_object_id);
          if (parkingspace_ptr != nullptr) {
            const auto& parkingspace_id = parkingspace_ptr->id();
            const auto& parkingspace_raw_id = parkingspace_id.id();
            if (polygon_id_set.find(parkingspace_raw_id) ==
                polygon_id_set.end()) {
              PolygonRoiPtr polygon_roi_ptr(new PolygonRoi());
              polygon_roi_ptr->polygon = parkingspace_ptr->polygon();
              polygon_roi_ptr->attribute.type =
                  PolygonType::PARKINGSPACE_POLYGON;
              polygon_roi_ptr->attribute.id = parkingspace_id;
              polygons_roi->push_back(polygon_roi_ptr);
              polygon_id_set.insert(parkingspace_raw_id);
            }
          }
        }
      }
    }
  }
  return 0;
}

void Roads::PostProcess() {
  for (const auto& road_ptr_pair : table_) {
    const auto& road_id = road_ptr_pair.second->id();
    for (const auto& road_section : road_ptr_pair.second->sections()) {
      const auto& section_id = road_section.id();
      for (const auto& lane_id : road_section.lane_id()) {
        const auto lane_info = lanes_->GetById(lane_id);
        if (lane_info != nullptr) {
          auto modifiable_lane_info = const_cast<LaneInfo*>(lane_info.get());
          modifiable_lane_info->set_road_id(road_id);
          modifiable_lane_info->set_section_id(section_id);
        } else {
          AFATAL << "Unknown lane id: " << lane_id.id();
        }
      }
    }
  }
}

int RSUs::GetForwardNearestItems(const apollo::common::PointENU& point,
                                 double distance, double central_heading,
                                 double max_heading_difference,
                                 std::vector<RSUInfoConstPtr>* rsus) const {
  CHECK_NOTNULL(rsus);

  rsus->clear();
  LaneInfoConstPtr lane_ptr = nullptr;
  apollo::common::math::Vec2d target_point(point.x(), point.y());

  double nearest_s = 0.0;
  double nearest_l = 0.0;
  if (lanes_.GetNearestWithHeading(target_point, distance, central_heading,
                                   max_heading_difference, &lane_ptr,
                                   &nearest_s, &nearest_l) == -1) {
    AERROR << "Fail to get nearest lanes";
    return -1;
  }

  if (lane_ptr == nullptr) {
    AERROR << "Fail to get nearest lanes";
    return -1;
  }

  double s = 0;
  double real_distance = distance + nearest_s;
  const std::string nearst_lane_id = lane_ptr->id().id();

  while (s < real_distance) {
    s += lane_ptr->total_length();
    std::vector<std::pair<double, JunctionInfoConstPtr>> overlap_junctions;
    double start_s = 0;
    for (size_t x = 0; x != lane_ptr->junctions().size(); ++x) {
      const auto overlap_ptr = lane_ptr->junctions()[x];
      const auto& overlap = overlap_ptr->inner_object();
      const auto& lane_raw_id = lane_ptr->id().id();
      for (int i = 0; i != overlap.object_size(); ++i) {
        const auto& overlap_object = overlap.object(i);
        const auto& overlap_object_id = overlap_object.id();
        if (overlap_object_id.id() == lane_raw_id) {
          start_s = overlap_object.lane_overlap_info().start_s();
          continue;
        }

        const auto junction_ptr = junctions_.GetById(overlap_object_id);
        CHECK_NOTNULL(junction_ptr);
        if (nearst_lane_id == lane_raw_id &&
            !junction_ptr->polygon().IsPointIn(target_point)) {
          if (nearest_s > start_s) {
            continue;
          }
        }

        overlap_junctions.push_back(std::make_pair(start_s, junction_ptr));
      }
    }

    std::sort(overlap_junctions.begin(), overlap_junctions.end());

    std::unordered_set<std::string> duplicate_checker;
    duplicate_checker.reserve(overlap_junctions.size() / 2);

    for (const auto& overlap_junction : overlap_junctions) {
      const auto& junction = overlap_junction.second;
      const auto& junction_raw_id = junction->id().id();
      if (duplicate_checker.count(junction_raw_id) > 0) {
        continue;
      }
      duplicate_checker.insert(junction_raw_id);

      for (const auto& overlap_id : junction->inner_object().overlap_id()) {
        OverlapInfoConstPtr overlap_ptr = overlaps_.GetById(overlap_id);
        CHECK_NOTNULL(overlap_ptr);
        const auto& overlap = overlap_ptr->inner_object();
        for (int i = 0; i != overlap.object_size(); ++i) {
          const auto& overlap_object = overlap.object(i);
          if (!overlap_object.has_rsu_overlap_info()) {
            continue;
          }

          const auto rsu_ptr = this->GetById(overlap_object.id());
          if (rsu_ptr != nullptr) {
            rsus->push_back(rsu_ptr);
          }
        }
      }

      if (!rsus->empty()) {
        break;
      }
    }

    if (!rsus->empty()) {
      break;
    }

    for (const auto succesor_lane_id :
         lane_ptr->inner_object().successor_id()) {
      LaneInfoConstPtr succesor_lane_ptr = lanes_.GetById(succesor_lane_id);
      if (lane_ptr->inner_object().successor_id_size() > 1) {
        if (succesor_lane_ptr->inner_object().turn() ==
            apollo::hdmap::Lane::NO_TURN) {
          lane_ptr = succesor_lane_ptr;
          break;
        }
      } else {
        lane_ptr = succesor_lane_ptr;
        break;
      }
    }
  }

  if (rsus->empty()) {
    return -1;
  }

  return 0;
}

}  // namespace objects
}  // namespace hdmap
}  // namespace apollo
