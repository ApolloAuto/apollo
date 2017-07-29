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

MapPathPoint make_map_path_point(double x, double y, double heading,
                                 std::vector<LaneWaypoint> waypoints) {
  return MapPathPoint({x, y}, heading, waypoints);
}
MapPathPoint make_map_path_point(double x, double y, double heading,
                                 const LaneWaypoint& waypoint) {
  std::vector<LaneWaypoint> waypoints;
  waypoints.push_back(waypoint);
  return MapPathPoint({x, y}, heading, waypoints);
}
MapPathPoint make_map_path_point(double x, double y, double heading) {
  return MapPathPoint({x, y}, heading);
}
MapPathPoint make_map_path_point(double x, double y) {
  return make_map_path_point(x, y, 0);
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

TEST(TestSuite, hdmap_line_path) {
  Lane lane;
  lane.mutable_id()->set_id("id");
  auto* line_segment =
      lane.mutable_central_curve()->add_segment()->mutable_line_segment();
  *line_segment->add_point() = make_point(0, 0, 0);
  *line_segment->add_point() = make_point(0, 3, 0);
  lane.set_length(3.0);
  *lane.add_left_sample() = make_sample(0.0, 4.0);
  *lane.add_left_sample() = make_sample(1.0, 5.0);
  *lane.add_left_sample() = make_sample(3.0, 6.0);
  *lane.add_right_sample() = make_sample(0.0, 7.0);
  *lane.add_right_sample() = make_sample(2.0, 8.0);
  *lane.add_right_sample() = make_sample(3.0, 5.0);

  apollo::hdmap::LaneInfoConstPtr lane_info(new LaneInfo(lane));

  const std::vector<MapPathPoint> points{
      make_map_path_point(0, 0, M_PI_2, LaneWaypoint(lane_info, 0)),
      make_map_path_point(0, 1, M_PI_2, LaneWaypoint(lane_info, 1)),
      make_map_path_point(0, 2, M_PI_2, LaneWaypoint(lane_info, 2)),
      make_map_path_point(0, 3, M_PI_2, LaneWaypoint(lane_info, 3))};
  const Path path(points, {}, 2.0);
  EXPECT_EQ(path.num_points(), 4);
  EXPECT_EQ(path.num_segments(), 3);
  EXPECT_NEAR(path.path_points()[0].x(), 0, 1e-6);
  EXPECT_NEAR(path.path_points()[0].y(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[0].x(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[0].y(), 1, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[0], 0, 1e-6);
  EXPECT_NEAR(path.path_points()[1].x(), 0, 1e-6);
  EXPECT_NEAR(path.path_points()[1].y(), 1, 1e-6);
  EXPECT_NEAR(path.unit_directions()[1].x(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[1].y(), 1, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[1], 1, 1e-6);
  EXPECT_NEAR(path.path_points()[2].x(), 0, 1e-6);
  EXPECT_NEAR(path.path_points()[2].y(), 2, 1e-6);
  EXPECT_NEAR(path.unit_directions()[2].x(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[2].y(), 1, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[2], 2, 1e-6);
  EXPECT_NEAR(path.path_points()[3].x(), 0, 1e-6);
  EXPECT_NEAR(path.path_points()[3].y(), 3, 1e-6);
  EXPECT_NEAR(path.unit_directions()[3].x(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[3].y(), 1, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[3], 3, 1e-6);
  EXPECT_EQ(path.segments().size(), 3);
  const auto* path_approximation = path.approximation();
  EXPECT_NEAR(path_approximation->max_error(), 2.0, 1e-6);
  EXPECT_EQ(path_approximation->original_ids().size(), 2);
  EXPECT_EQ(path_approximation->original_ids()[0], 0);
  EXPECT_EQ(path_approximation->original_ids()[1], 3);
  EXPECT_EQ(path.lane_segments().size(), 3);

  EXPECT_EQ(path.lane_segments_to_next_point().size(), 3);
  EXPECT_EQ(path.lane_segments_to_next_point()[0].lane->id().id(), "id");
  EXPECT_NEAR(path.lane_segments_to_next_point()[0].start_s, 0.0, 1e-6);
  EXPECT_NEAR(path.lane_segments_to_next_point()[0].end_s, 1.0, 1e-6);
  EXPECT_EQ(path.lane_segments_to_next_point()[1].lane->id().id(), "id");
  EXPECT_NEAR(path.lane_segments_to_next_point()[1].start_s, 1.0, 1e-6);
  EXPECT_NEAR(path.lane_segments_to_next_point()[1].end_s, 2.0, 1e-6);
  EXPECT_EQ(path.lane_segments_to_next_point()[2].lane->id().id(), "id");
  EXPECT_NEAR(path.lane_segments_to_next_point()[2].start_s, 2.0, 1e-6);
  EXPECT_NEAR(path.lane_segments_to_next_point()[2].end_s, 3.0, 1e-6);

  MapPathPoint point = path.get_smooth_point({1, 0.5});
  EXPECT_NEAR(point.x(), 0, 1e-6);
  EXPECT_NEAR(point.y(), 1.5, 1e-6);
  point = path.get_smooth_point(2.3);
  EXPECT_NEAR(point.x(), 0, 1e-6);
  EXPECT_NEAR(point.y(), 2.3, 1e-6);
  point = path.get_smooth_point(-0.5);
  EXPECT_NEAR(point.x(), 0, 1e-6);
  EXPECT_NEAR(point.y(), 0, 1e-6);
  point = path.get_smooth_point(10.0);
  EXPECT_NEAR(point.x(), 0, 1e-6);
  EXPECT_NEAR(point.y(), 3, 1e-6);

  EXPECT_NEAR(path.get_s_from_index({1, 0.4}), 1.4, 1e-6);
  EXPECT_EQ(path.get_index_from_s(2.6).id, 2);
  EXPECT_NEAR(path.get_index_from_s(2.6).offset, 0.6, 1e-6);

  double accumulate_s;
  double lateral;
  double distance;
  EXPECT_TRUE(
      path.get_nearest_point({0.3, -1}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 0.0, 1e-6);
  EXPECT_NEAR(lateral, -0.3, 1e-6);
  EXPECT_NEAR(distance, hypot(0.3, 1.0), 1e-6);
  EXPECT_TRUE(
      path.get_nearest_point({-0.5, 1.5}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 1.5, 1e-6);
  EXPECT_NEAR(lateral, 0.5, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);
  EXPECT_TRUE(
      path.get_nearest_point({0.0, 3.5}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 3.0, 1e-6);
  EXPECT_NEAR(lateral, 0.0, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);

  EXPECT_TRUE(
      path.get_projection({0.3, -1}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, -1.0, 1e-6);
  EXPECT_NEAR(lateral, -0.3, 1e-6);
  EXPECT_NEAR(distance, hypot(0.3, 1.0), 1e-6);
  EXPECT_TRUE(
      path.get_projection({-0.5, 1.5}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 1.5, 1e-6);
  EXPECT_NEAR(lateral, 0.5, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);
  EXPECT_TRUE(
      path.get_projection({0.0, 3.5}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 3.5, 1e-6);
  EXPECT_NEAR(lateral, 0.0, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);

  EXPECT_NEAR(path.get_left_width(-0.5), 4.0, 1e-6);
  EXPECT_NEAR(path.get_left_width(0.0), 4.0, 1e-6);
  EXPECT_NEAR(path.get_left_width(0.5), 4.5, 1e-6);
  EXPECT_NEAR(path.get_left_width(1.0), 5.0, 1e-6);
  EXPECT_NEAR(path.get_left_width(1.5), 5.25, 1e-6);
  EXPECT_NEAR(path.get_left_width(2.0), 5.5, 1e-6);
  EXPECT_NEAR(path.get_left_width(2.5), 5.75, 1e-6);
  EXPECT_NEAR(path.get_left_width(3.0), 6.0, 1e-6);
  EXPECT_NEAR(path.get_left_width(3.5), 6.0, 1e-6);

  EXPECT_NEAR(path.get_right_width(-0.5), 7.0, 1e-6);
  EXPECT_NEAR(path.get_right_width(0.0), 7.0, 1e-6);
  EXPECT_NEAR(path.get_right_width(0.5), 7.25, 1e-6);
  EXPECT_NEAR(path.get_right_width(1.0), 7.5, 1e-6);
  EXPECT_NEAR(path.get_right_width(1.5), 7.75, 1e-6);
  EXPECT_NEAR(path.get_right_width(2.0), 8.0, 1e-6);
  EXPECT_NEAR(path.get_right_width(2.5), 6.5, 1e-6);
  EXPECT_NEAR(path.get_right_width(3.0), 5.0, 1e-6);
  EXPECT_NEAR(path.get_right_width(3.5), 5.0, 1e-6);
}

}  // hdmap
}  // apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
