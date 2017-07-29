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

#include "modules/map/pnc_map/path.h"

#include <algorithm>
#include <vector>

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "modules/map/proto/routing.pb.h"

#include "modules/map/hdmap/hdmap.h"

using Id = apollo::hdmap::Id;
using Lane = apollo::hdmap::Lane;
using LaneSampleAssociation = apollo::hdmap::LaneSampleAssociation;
using Point = apollo::hdmap::Point;
using RoutingRequest = apollo::hdmap::RoutingRequest;
using RoutingResult = apollo::hdmap::RoutingResult;
using MapPathPoint = apollo::hdmap::MapPathPoint;
using AABox2d = apollo::common::math::AABox2d;
using Box2d = apollo::common::math::Box2d;
using Polygon2d = apollo::common::math::Polygon2d;
using Vec2d = apollo::common::math::Vec2d;

DECLARE_bool(validate_routing_result_for_trajectories);

namespace apollo {
namespace hdmap {
namespace {

const std::string kMapFilename = "../data/example_intersection.graph";
const std::string kMapBinaryFilename = "../data/example_intersection.bin";
const std::string kSunnyvaleMapFilename = "../data/sunnyvale_map.txt";
const std::string kSunnyvaleMapBinaryFilename = "../data/sunnyvale_map.bin";
const std::string kSunnyvaleMapBinaryFilename_2 = "../data/sunnyvale_map_2.bin";
const std::string kSunnyvaleGarageMapFilename =
    "../data/sunnyvale_garage_map.txt";
const std::string kSunnyvaleGarageMapBinaryFilename =
    "../data/sunnyvale_garage_map.bin";

Point make_point(double x, double y, double z) {
  Point pt;
  pt.set_x(x);
  pt.set_y(y);
  pt.set_z(z);
  return pt;
}

Id make_lane_id(const std::string& id) {
  Id lane_id;
  lane_id.set_id(id);
  return lane_id;
}

Lane make_default_lane() {
  Lane lane;
  *lane.mutable_id() = make_lane_id("test");
  auto* segment =
      lane.mutable_central_curve()->add_segment()->mutable_line_segment();
  *segment->add_point() = make_point(0, 0, 0);
  *segment->add_point() = make_point(1, 1, 1);
  return lane;
}

LaneSampleAssociation make_sample(double s, double width) {
  LaneSampleAssociation sample;
  sample.set_s(s);
  sample.set_width(width);
  return sample;
}

const LaneInfo kDefaultaLaneInfo(make_default_lane());

MapPathPoint make_trajectory_point(double x, double y, double heading,
                                   std::vector<LaneWaypoint> waypoints) {
  return MapPathPoint({x, y}, heading, waypoints);
}
MapPathPoint make_trajectory_point(double x, double y, double heading,
                                   const LaneWaypoint& waypoint) {
  std::vector<LaneWaypoint> waypoints;
  waypoints.push_back(waypoint);
  return MapPathPoint({x, y}, heading, waypoints);
}
MapPathPoint make_trajectory_point(double x, double y, double heading) {
  return MapPathPoint({x, y}, heading);
}
MapPathPoint make_trajectory_point(double x, double y) {
  return make_trajectory_point(x, y, 0);
}

int random_int(int s, int t) {
  if (s >= t) {
    return s;
  }
  return s + rand() % (t - s + 1);
}

double random_double(double s, double t) {
  return s + (t - s) / 16383.0 * (rand() & 16383);
}

template <class T>
std::string to_string(const T& val) {
  std::stringstream ss;
  ss << val;
  ss.flush();
  return ss.str();
}

}  // namespace

}  // hdmap
}  // apollo
