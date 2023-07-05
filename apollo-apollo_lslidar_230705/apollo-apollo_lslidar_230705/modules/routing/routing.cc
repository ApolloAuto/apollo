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

#include "modules/routing/routing.h"

#include <limits>
#include <unordered_map>

#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

using apollo::common::ErrorCode;
using apollo::common::PointENU;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

apollo::common::Status Routing::Init() {
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));

  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  return apollo::common::Status::OK();
}

apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}

std::vector<RoutingRequest> Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  std::vector<RoutingRequest> fixed_requests;
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;
  RoutingRequest fixed_request(routing_request);
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    if (lane_waypoint.has_id()) {
      continue;
    }

    // fill lane info when missing
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    constexpr double kRadius = 0.3;
    for (int i = 0; i < 20; ++i) {
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      if (lanes.size() > 0) {
        break;
      }
    }
    if (lanes.empty()) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return fixed_requests;  // return empty vector
    }
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);
      if (j == 0) {
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }
    }
  }
  // first routing_request
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
  for (const auto& m : additional_lane_waypoint_map) {
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }

  for (const auto& fixed_request : fixed_requests) {
    ADEBUG << "Fixed routing request:" << fixed_request.DebugString();
  }
  return fixed_requests;
}

bool Routing::GetParkingID(const PointENU& parking_point,
                           std::string* parking_space_id) {
  // search current parking space id associated with parking point.
  constexpr double kDistance = 0.01;  // meter
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (hdmap_->GetParkingSpaces(parking_point, kDistance, &parking_spaces) ==
      0) {
    *parking_space_id = parking_spaces.front()->id().id();
    return true;
  }
  return false;
}

bool Routing::FillParkingID(RoutingResponse* routing_response) {
  const auto& routing_request = routing_response->routing_request();
  const bool has_parking_info = routing_request.has_parking_info();
  const bool has_parking_id =
      has_parking_info && routing_request.parking_info().has_parking_space_id();
  // return early when has parking_id
  if (has_parking_id) {
    return true;
  }
  // set parking space ID when
  //  has parking info && has parking point && NOT has parking space id && get
  //  ID successfully
  if (has_parking_info && routing_request.parking_info().has_parking_point()) {
    const PointENU parking_point =
        routing_request.parking_info().parking_point();
    std::string parking_space_id;
    if (GetParkingID(parking_point, &parking_space_id)) {
      routing_response->mutable_routing_request()
          ->mutable_parking_info()
          ->set_parking_space_id(parking_space_id);
      return true;
    }
  }
  ADEBUG << "Failed to fill parking ID";
  return false;
}

bool Routing::SupplementParkingRequest(
    RoutingResponse* const routing_response) const {
  const auto& routing_request = routing_response->routing_request();
  const bool has_parking_info = routing_request.has_parking_info();
  if (!has_parking_info) {
    return true;
  }

  // 1. Get the nearest lane along the parking spot, called "parking spot
  // lane". Calculate the center point of the parking spot, to find out the
  // lane with which the parking spot overlaps.
  const auto& points = routing_request.parking_info().corner_point();
  double headings[4];
  const common::PointENU* corner_points[4];
  for (size_t i = 0; i < 4; i++) {
    corner_points[i] = &(points.point().at(i));
  }
  headings[0] = std::atan2(corner_points[1]->y() - corner_points[0]->y(),
                           corner_points[1]->x() - corner_points[0]->x());
  headings[1] = std::atan2(corner_points[2]->y() - corner_points[1]->y(),
                           corner_points[2]->x() - corner_points[1]->x());
  headings[2] = headings[0] + M_PI;
  headings[3] = headings[1] + M_PI;
  // The distance range when searching the lanes along the parking spot.
  static const double kMaxHeadingDistance = M_PI_4;
  double distance_search_range = 5.0;
  double nearest_s = std::numeric_limits<double>::max();
  double nearest_l = std::numeric_limits<double>::max();
  hdmap::LaneInfoConstPtr nearest_lane;
  bool has_found_nearest_lane = false;
  // Get all the object overlap with the parking spot.
  std::vector<std::string> overlap_object_ids;
  const auto parking_space_id =
      hdmap::MakeMapId(routing_request.parking_info().parking_space_id());
  GetAllOverlapObjectIds(parking_space_id, &overlap_object_ids);
  for (size_t i = 0; i < 4; i++) {
    double temp_nearest_s;
    double temp_nearest_l;
    hdmap::LaneInfoConstPtr temp_nearest_lane;
    common::PointENU temp_point = common::util::PointFactory::ToPointENU(
        corner_points[i]->x(), corner_points[i]->y());
    // Find the nearest lane with the current heading.
    if (0 != hdmap_->GetNearestLaneWithHeading(
                 temp_point, distance_search_range, headings[i],
                 kMaxHeadingDistance, &temp_nearest_lane, &temp_nearest_s,
                 &temp_nearest_l) ||
        0 == std::count(overlap_object_ids.begin(), overlap_object_ids.end(),
                        temp_nearest_lane->id().id())) {
      // Find the nearest lane with the opposite heading on the same segment.
      size_t next_index = i + 1;
      if (next_index > 3) {
        next_index = 0;
      }
      temp_point = common::util::PointFactory::ToPointENU(
          corner_points[next_index]->x(), corner_points[next_index]->y());
      if (0 != hdmap_->GetNearestLaneWithHeading(
                   temp_point, distance_search_range, headings[i] - M_PI,
                   kMaxHeadingDistance, &temp_nearest_lane, &temp_nearest_s,
                   &temp_nearest_l) ||
          0 == std::count(overlap_object_ids.begin(), overlap_object_ids.end(),
                          temp_nearest_lane->id().id())) {
        continue;
      }
    }
    if (std::fabs(nearest_l) > std::fabs(temp_nearest_l)) {
      nearest_l = temp_nearest_l;
      nearest_lane = temp_nearest_lane;
      nearest_s = temp_nearest_s;
      if (distance_search_range > std::fabs(temp_nearest_l)) {
        distance_search_range = std::fabs(temp_nearest_l);
      }
      has_found_nearest_lane = true;
    }
  }
  if (!has_found_nearest_lane) {
    AWARN << "Supplement Parking Request but no nearest lane found!";
    return true;
  }

  // 2. If the RoadSegments of the routing_response contain the "parking spot
  // lane", make sure the distance range of the segment contains the border
  // of the parking spot
  for (auto& road : *(routing_response->mutable_road())) {
    for (auto& passage_region : *(road.mutable_passage())) {
      for (auto& segment : *(passage_region.mutable_segment())) {
        if (segment.id() == nearest_lane->id().id()) {
          // Make sure the segment range contain the all the corner points of
          // the parking spot.
          if (segment.start_s() > nearest_s) {
            segment.set_start_s(nearest_s);
          }
          return true;
        }
      }
    }
  }

  // 3. In case the RoadSegments of the routing_response don't contain the
  // "parking spot lane", set it as the start point of the routing request
  // and do the rerouting.
  auto waypoints =
      routing_response->mutable_routing_request()->mutable_waypoint();
  // Add the point to the beginning of the waypoints list.
  waypoints->Add();
  for (size_t i = waypoints->size() - 1; i > 0; --i) {
    waypoints->SwapElements(i, i - 1);
  }
  auto first_way_point = waypoints->Mutable(0);
  first_way_point->set_id(nearest_lane->id().id());
  first_way_point->set_s(nearest_s);
  common::PointENU point = nearest_lane->GetSmoothPoint(nearest_s);
  auto* pose = first_way_point->mutable_pose();
  pose->set_x(point.x());
  pose->set_y(point.y());

  RoutingResponse routing_response_temp;
  if (navigator_ptr_->SearchRoute(routing_response->routing_request(),
                                  &routing_response_temp)) {
    routing_response->CopyFrom(routing_response_temp);
    return true;
  }
  return false;
}

void Routing::GetAllOverlapObjectIds(
    const hdmap::Id& parking_spot_id,
    std::vector<std::string> *object_ids) const {
  object_ids->clear();
  auto parking_spot_info = hdmap_->GetParkingSpaceById(parking_spot_id);
  if (nullptr == parking_spot_info) {
    return;
  }
  auto overlap_ids = parking_spot_info->parking_space().overlap_id();
  for (auto& id : overlap_ids) {
    auto overlap_info = hdmap_->GetOverlapById(id);
    if (nullptr == overlap_info) {
      continue;
    }
    auto overlap = overlap_info->overlap();
    for (auto object : overlap.object()) {
      object_ids->emplace_back(object.id().id());
    }
  }
}

bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request,
                      RoutingResponse* const routing_response) {
  CHECK_NOTNULL(routing_response);
  AINFO << "Get new routing request:" << routing_request->DebugString();

  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);
  double min_routing_length = std::numeric_limits<double>::max();
  for (const auto& fixed_request : fixed_requests) {
    RoutingResponse routing_response_temp;
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
      const double routing_length =
          routing_response_temp.measurement().distance();
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
    FillParkingID(routing_response);
  }
  if (min_routing_length < std::numeric_limits<double>::max() &&
      SupplementParkingRequest(routing_response)) {
    monitor_logger_buffer_.INFO("Routing success!");
    return true;
  }

  AERROR << "Failed to search route with navigator.";
  monitor_logger_buffer_.WARN("Routing failed! " +
                              routing_response->status().msg());
  return false;
}

}  // namespace routing
}  // namespace apollo
