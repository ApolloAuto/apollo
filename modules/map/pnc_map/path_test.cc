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
using Point = apollo::common::PointENU;
using RoutingRequest = apollo::hdmap::RoutingRequest;
using RoutingResult = apollo::hdmap::RoutingResult;
using MapPathPoint = apollo::hdmap::MapPathPoint;
using AABox2d = apollo::common::math::AABox2d;
using Box2d = apollo::common::math::Box2d;
using Polygon2d = apollo::common::math::Polygon2d;
using Vec2d = apollo::common::math::Vec2d;
using LaneWaypoint = apollo::hdmap::LaneWaypoint;
using LaneInfoConstPtr = apollo::hdmap::LaneInfoConstPtr;

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

  LaneInfoConstPtr lane_info(new LaneInfo(lane));

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

TEST(TestSuite, hdmap_curvy_path) {
  const std::vector<MapPathPoint> points{
      make_map_path_point(2, 0), make_map_path_point(2, 1),
      make_map_path_point(1, 2), make_map_path_point(0, 2)};
  Path path(points, {}, 2.0);
  EXPECT_EQ(path.num_points(), 4);
  EXPECT_EQ(path.num_segments(), 3);
  EXPECT_NEAR(path.path_points()[0].x(), 2, 1e-6);
  EXPECT_NEAR(path.path_points()[0].y(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[0].x(), 0, 1e-6);
  EXPECT_NEAR(path.unit_directions()[0].y(), 1, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[0], 0, 1e-6);
  EXPECT_NEAR(path.path_points()[1].x(), 2, 1e-6);
  EXPECT_NEAR(path.path_points()[1].y(), 1, 1e-6);
  EXPECT_NEAR(path.unit_directions()[1].x(), -0.5 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(path.unit_directions()[1].y(), 0.5 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(path.accumulated_s()[1], 1, 1e-6);
  EXPECT_NEAR(path.path_points()[2].x(), 1, 1e-6);
  EXPECT_NEAR(path.path_points()[2].y(), 2, 1e-6);
  EXPECT_NEAR(path.unit_directions()[2].x(), -1, 1e-6);
  EXPECT_NEAR(path.unit_directions()[2].y(), 0, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[2], 1 + sqrt(2.0), 1e-6);
  EXPECT_NEAR(path.path_points()[3].x(), 0, 1e-6);
  EXPECT_NEAR(path.path_points()[3].y(), 2, 1e-6);
  EXPECT_NEAR(path.unit_directions()[3].x(), -1, 1e-6);
  EXPECT_NEAR(path.unit_directions()[3].y(), 0, 1e-6);
  EXPECT_NEAR(path.accumulated_s()[3], 2 + sqrt(2.0), 1e-6);
  EXPECT_EQ(path.segments().size(), 3);
  const auto* path_approximation = path.approximation();
  EXPECT_NEAR(path_approximation->max_error(), 2.0, 1e-6);
  EXPECT_EQ(path_approximation->original_ids().size(), 2);
  EXPECT_EQ(path_approximation->original_ids()[0], 0);
  EXPECT_EQ(path_approximation->original_ids()[1], 3);
  EXPECT_EQ(path.lane_segments().size(), 0);

  double accumulate_s;
  double lateral;
  double distance;
  EXPECT_TRUE(
      path.get_nearest_point({1.5, 0.5}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 0.5, 1e-6);
  EXPECT_NEAR(lateral, 0.5, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);
  EXPECT_TRUE(
      path.get_nearest_point({2.5, 1.1}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 1.0, 1e-6);
  EXPECT_NEAR(distance, hypot(0.5, 0.1), 1e-6);
  EXPECT_TRUE(
      path.get_nearest_point({1.6, 1.6}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 1.0 + 0.5 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(lateral, -0.1 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(distance, 0.1 * sqrt(2.0), 1e-6);

  EXPECT_TRUE(
      path.get_projection({1.5, 0.5}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 0.5, 1e-6);
  EXPECT_NEAR(lateral, 0.5, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);
  EXPECT_TRUE(
      path.get_projection({2.5, 1.1}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 1.0, 1e-6);
  EXPECT_NEAR(distance, hypot(0.5, 0.1), 1e-6);
  EXPECT_TRUE(
      path.get_projection({1.6, 1.6}, &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, 1.0 + 0.5 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(lateral, -0.1 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(distance, 0.1 * sqrt(2.0), 1e-6);

  EXPECT_NEAR(path.get_s_from_index({-1, 0.5}), 0.0, 1e-6);
  EXPECT_NEAR(path.get_s_from_index({0, 0.5}), 0.5, 1e-6);
  EXPECT_NEAR(path.get_s_from_index({1, 0.5}), 1.5, 1e-6);
  EXPECT_NEAR(path.get_s_from_index({2, 0.5}), 1.5 + sqrt(2.0), 1e-6);
  EXPECT_NEAR(path.get_s_from_index({3, 0.0}), 2 + sqrt(2.0), 1e-6);
  EXPECT_NEAR(path.get_s_from_index({4, 0.0}), 2 + sqrt(2.0), 1e-6);

  const auto& traj_points = path.path_points();
  for (size_t i = 0; i < traj_points.size(); ++i) {
    Vec2d point{traj_points[i].x(), traj_points[i].y()};
    double heading = 0;
    path.get_heading_along_path(point, &heading);
    EXPECT_NEAR(heading, traj_points[i].heading(), 1e-5);
  }

  // Test move constructor.
  Path other_path(std::move(path));
  // TODO: check why path.approximation() is not nullptr here.
  const auto* other_path_approximation = other_path.approximation();
  EXPECT_NEAR(other_path_approximation->max_error(), 2.0, 1e-6);
  EXPECT_EQ(other_path_approximation->original_ids().size(), 2);
  EXPECT_EQ(other_path_approximation->original_ids()[0], 0);
  EXPECT_EQ(other_path_approximation->original_ids()[1], 3);

  EXPECT_TRUE(other_path.get_projection({1.5, 0.5}, &accumulate_s, &lateral,
                                        &distance));
  EXPECT_NEAR(accumulate_s, 0.5, 1e-6);
  EXPECT_NEAR(lateral, 0.5, 1e-6);
  EXPECT_NEAR(distance, 0.5, 1e-6);
  EXPECT_TRUE(other_path.get_projection({2.5, 1.1}, &accumulate_s, &lateral,
                                        &distance));
  EXPECT_NEAR(accumulate_s, 1.0, 1e-6);
  EXPECT_NEAR(distance, hypot(0.5, 0.1), 1e-6);
  EXPECT_TRUE(other_path.get_projection({1.6, 1.6}, &accumulate_s, &lateral,
                                        &distance));
  EXPECT_NEAR(accumulate_s, 1.0 + 0.5 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(lateral, -0.1 * sqrt(2.0), 1e-6);
  EXPECT_NEAR(distance, 0.1 * sqrt(2.0), 1e-6);
}

TEST(TestSuite, hdmap_circle_path) {
  const double kRadius = 50.0;
  const int kNumSegments = 100;
  std::vector<MapPathPoint> points;
  for (int i = 0; i <= kNumSegments; ++i) {
    const double p =
        M_PI_2 * static_cast<double>(i) / static_cast<double>(kNumSegments);
    points.push_back(make_map_path_point(kRadius * cos(p), kRadius * sin(p)));
  }
  const Path path(points, {}, 2.0);
  EXPECT_EQ(path.num_points(), kNumSegments + 1);
  EXPECT_EQ(path.num_segments(), kNumSegments);
  EXPECT_EQ(path.path_points().size(), kNumSegments + 1);
  EXPECT_EQ(path.lane_segments().size(), 0);
  EXPECT_EQ(path.segments().size(), kNumSegments);
  const auto* path_approximation = path.approximation();
  EXPECT_NEAR(path_approximation->max_error(), 2.0, 1e-6);
  EXPECT_EQ(path_approximation->original_ids().size(), 4);
  EXPECT_EQ(path_approximation->original_ids()[0], 0);
  EXPECT_EQ(path_approximation->original_ids()[1], 36);
  EXPECT_EQ(path_approximation->original_ids()[2], 72);
  EXPECT_EQ(path_approximation->original_ids()[3], 100);
  const double total_length = path.accumulated_s().back();

  const double kLargeEps = 0.1;
  double accumulate_s;
  double lateral;
  double distance;
  EXPECT_TRUE(path.get_projection({kRadius + 1, -1}, &accumulate_s, &lateral,
                                  &distance));
  EXPECT_NEAR(accumulate_s, -1.0, kLargeEps);
  EXPECT_NEAR(lateral, -1.0, kLargeEps);
  EXPECT_NEAR(distance, sqrt(2.0), 1e-6);

  EXPECT_TRUE(path.get_projection({-1, kRadius - 1}, &accumulate_s, &lateral,
                                  &distance));
  EXPECT_NEAR(accumulate_s, total_length + 1.0, kLargeEps);
  EXPECT_NEAR(lateral, 1.0, kLargeEps);
  EXPECT_NEAR(distance, sqrt(2.0), 1e-6);

  EXPECT_TRUE(
      path.get_projection({kRadius / sqrt(2.0) + 1, kRadius / sqrt(2.0) + 1},
                          &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, total_length / 2.0, 1e-6);
  EXPECT_NEAR(lateral, -sqrt(2.0), 1e-6);
  EXPECT_NEAR(distance, sqrt(2.0), 1e-6);

  EXPECT_TRUE(path.get_projection({kRadius / sqrt(2.0), kRadius / sqrt(2.0)},
                                  &accumulate_s, &lateral, &distance));
  EXPECT_NEAR(accumulate_s, total_length / 2.0, 1e-6);
  EXPECT_NEAR(lateral, 0.0, 1e-6);
  EXPECT_NEAR(distance, 0.0, 1e-6);

  // Randomly generated test cases on path.approxmiation.
  const Path path_no_approximation(points, {});
  for (int case_id = 0; case_id < 10000; ++case_id) {
    const double x = random_double(-kRadius * 0.5, kRadius * 1.5);
    const double y = random_double(-kRadius * 0.5, kRadius * 1.5);
    EXPECT_TRUE(
        path.get_nearest_point({x, y}, &accumulate_s, &lateral, &distance));

    double other_accumulate_s;
    double other_lateral;
    double other_distance;
    EXPECT_TRUE(path_no_approximation.get_nearest_point(
        {x, y}, &other_accumulate_s, &other_lateral, &other_distance));

    EXPECT_NEAR(distance, other_distance, 1e-6);
    EXPECT_NEAR(path.get_smooth_point(accumulate_s).DistanceTo({x, y}),
                distance, 1e-6);
  }

  // Test path.get_smooth_point and get_s_from_index
  for (int case_id = -10; case_id <= 80; ++case_id) {
    const double ratio = static_cast<double>(case_id) / 70.0;
    const double s = path.length() * ratio;
    const auto index = path.get_index_from_s(s);

    const double angle = M_PI_2 / static_cast<double>(kNumSegments);
    ;
    const double length = kRadius * sin(angle / 2.0) * 2.0;
    if (s <= 0.0) {
      EXPECT_EQ(0, index.id);
      EXPECT_NEAR(0.0, index.offset, 1e-6);
    } else if (s >= path.length()) {
      EXPECT_EQ(kNumSegments, index.id);
      EXPECT_NEAR(0.0, index.offset, 1e-6);
    } else {
      EXPECT_EQ(static_cast<int>(s / length), index.id);
      EXPECT_NEAR(fmod(s, length), index.offset, 1e-6);
    }
    const MapPathPoint point = path.get_smooth_point(s);
    Vec2d expected_point = points[index.id];
    if (index.id + 1 < static_cast<int>(points.size())) {
      Vec2d direction = points[index.id + 1] - points[index.id];
      direction.Normalize();
      expected_point += direction * index.offset;
    }
    EXPECT_NEAR(expected_point.x(), point.x(), 1e-6);
    EXPECT_NEAR(expected_point.y(), point.y(), 1e-6);
  }

  // Test get_width, get_left_width, get_right_width
  double delta_s = 0.1;
  double cur_s = 0.0;
  while (cur_s < path.accumulated_s().back()) {
    double left_width = 0.0;
    double right_width = 0.0;
    EXPECT_TRUE(path.get_width(cur_s, &left_width, &right_width));
    EXPECT_NEAR(left_width, path.get_left_width(cur_s), 1e-6);
    EXPECT_NEAR(right_width, path.get_right_width(cur_s), 1e-6);
    cur_s += delta_s;
  }
}

TEST(TestSuite, hdmap_jerky_path) {
  const int kNumPaths = 100;
  const int kCasesPerPath = 1000;
  for (int path_id = 0; path_id < kNumPaths; ++path_id) {
    const int num_segments = random_int(50, 100);
    const double average_segment_length = random_double(0.5, 5.0);
    const double max_y = random_double(0.5, 10.0);

    std::vector<MapPathPoint> points;
    double sum_x = 0;
    for (int i = 0; i <= num_segments; ++i) {
      points.push_back(
          make_map_path_point(sum_x, random_double(-max_y, max_y)));
      sum_x += random_double(average_segment_length * 0.1,
                             average_segment_length * 1.9);
    }
    const double angle = random_double(0, M_PI);
    const double cos_angle = cos(angle);
    const double sin_angle = sin(angle);
    for (auto& point : points) {
      const double new_x = point.x() * cos_angle - point.y() * sin_angle;
      const double new_y = point.x() * sin_angle + point.y() * cos_angle;
      point.set_x(new_x);
      point.set_y(new_y);
    }
    const Path path(points, {}, 2.0);
    EXPECT_EQ(path.num_points(), num_segments + 1);
    EXPECT_EQ(path.num_segments(), num_segments);
    EXPECT_EQ(path.path_points().size(), num_segments + 1);
    EXPECT_EQ(path.lane_segments().size(), 0);
    EXPECT_EQ(path.segments().size(), num_segments);
    const auto* path_approximation = path.approximation();
    EXPECT_NEAR(path_approximation->max_error(), 2.0, 1e-6);

    double accumulate_s;
    double lateral;
    double distance;
    EXPECT_TRUE(
        path.get_projection(points[0], &accumulate_s, &lateral, &distance));
    EXPECT_NEAR(accumulate_s, 0.0, 1e-6);
    EXPECT_NEAR(lateral, 0.0, 1e-6);
    EXPECT_NEAR(distance, 0.0, 1e-6);
    EXPECT_TRUE(path.get_projection(points[num_segments], &accumulate_s,
                                    &lateral, &distance));
    EXPECT_NEAR(accumulate_s, path.length(), 1e-6);
    EXPECT_NEAR(lateral, 0.0, 1e-6);
    EXPECT_NEAR(distance, 0.0, 1e-6);

    std::vector<Vec2d> original_points;
    for (const auto& point : points) {
      original_points.push_back(point);
    }
    const AABox2d box(original_points);
    const Path path_no_approximation(points, {});
    for (int case_id = 0; case_id < kCasesPerPath; ++case_id) {
      const double x = random_double(box.min_x(), box.max_x());
      const double y = random_double(box.min_y(), box.max_y());
      EXPECT_TRUE(
          path.get_nearest_point({x, y}, &accumulate_s, &lateral, &distance));

      double other_accumulate_s;
      double other_lateral;
      double other_distance;
      EXPECT_TRUE(path_no_approximation.get_nearest_point(
          {x, y}, &other_accumulate_s, &other_lateral, &other_distance));

      EXPECT_NEAR(distance, other_distance, 1e-6);
      EXPECT_NEAR(path.get_smooth_point(accumulate_s).DistanceTo({x, y}),
                  distance, 1e-6);
    }
  }
}

TEST(TestSuite, hdmap_s_path) {
  std::vector<MapPathPoint> points;
  const double kRadius = 50.0;
  const int kNumSegments = 100;
  for (int i = 0; i <= kNumSegments; ++i) {
    if (i <= kNumSegments / 2) {
      const double p = -M_PI_2 +
                       2.0 * M_PI * static_cast<double>(i) /
                           static_cast<double>(kNumSegments);
      points.push_back(
          make_map_path_point(kRadius * cos(p), kRadius * (sin(p) - 1.0)));
    } else {
      const double p = M_PI_2 -
                       2.0 * M_PI * static_cast<double>(i) /
                           static_cast<double>(kNumSegments);
      points.push_back(
          make_map_path_point(kRadius * cos(p), kRadius * (sin(p) + 1.0)));
    }
  }
  const Path path(points, {}, 2.0);
  EXPECT_EQ(path.num_points(), kNumSegments + 1);
  EXPECT_EQ(path.num_segments(), kNumSegments);
  EXPECT_EQ(path.path_points().size(), kNumSegments + 1);
  EXPECT_EQ(path.lane_segments().size(), 0);
  EXPECT_EQ(path.segments().size(), kNumSegments);
  const auto* path_approximation = path.approximation();
  EXPECT_NEAR(path_approximation->max_error(), 2.0, 1e-6);
  EXPECT_EQ(path_approximation->original_ids().size(), 12);
  EXPECT_EQ(path_approximation->original_ids()[0], 0);
  EXPECT_EQ(path_approximation->original_ids()[1], 9);
  EXPECT_EQ(path_approximation->original_ids()[2], 18);
  EXPECT_EQ(path_approximation->original_ids()[3], 27);
  EXPECT_EQ(path_approximation->original_ids()[4], 36);
  EXPECT_EQ(path_approximation->original_ids()[5], 45);
  EXPECT_EQ(path_approximation->original_ids()[6], 57);
  EXPECT_EQ(path_approximation->original_ids()[7], 66);
  EXPECT_EQ(path_approximation->original_ids()[8], 75);
  EXPECT_EQ(path_approximation->original_ids()[9], 84);
  EXPECT_EQ(path_approximation->original_ids()[10], 93);
  EXPECT_EQ(path_approximation->original_ids()[11], 100);
  const double total_length = path.accumulated_s().back();

  const double kLargeEps = 0.1;
  double accumulate_s;
  double lateral;
  double distance;
  EXPECT_TRUE(path.get_projection({-1, -2.0 * kRadius - 1}, &accumulate_s,
                                  &lateral, &distance));
  EXPECT_NEAR(accumulate_s, -1.0, kLargeEps);
  EXPECT_NEAR(lateral, -1.0, kLargeEps);
  EXPECT_NEAR(distance, sqrt(2.0), 1e-6);

  EXPECT_TRUE(path.get_projection({1, 2.0 * kRadius + 1}, &accumulate_s,
                                  &lateral, &distance));
  EXPECT_NEAR(accumulate_s, total_length + 1.0, kLargeEps);
  EXPECT_NEAR(lateral, 1.0, kLargeEps);
  EXPECT_NEAR(distance, sqrt(2.0), 1e-6);

  const Path path_no_approximation(points, {});
  for (int case_id = 0; case_id < 10000; ++case_id) {
    const double x = random_double(-kRadius * 1.5, kRadius * 1.5);
    const double y = random_double(-kRadius * 2.5, kRadius * 2.5);
    EXPECT_TRUE(
        path.get_nearest_point({x, y}, &accumulate_s, &lateral, &distance));

    double other_accumulate_s;
    double other_lateral;
    double other_distance;
    EXPECT_TRUE(path_no_approximation.get_nearest_point(
        {x, y}, &other_accumulate_s, &other_lateral, &other_distance));

    EXPECT_NEAR(distance, other_distance, 1e-6);
    EXPECT_NEAR(path.get_smooth_point(accumulate_s).DistanceTo({x, y}),
                distance, 1e-6);
  }
}

TEST(TestSuite, hdmap_path_get_smooth_point) {
  const double kRadius = 50.0;
  const int kNumSegments = 100;
  std::vector<MapPathPoint> points;
  for (int i = 0; i <= kNumSegments; ++i) {
    if (i <= kNumSegments / 2) {
      const double p = -M_PI_2 +
                       2.0 * M_PI * static_cast<double>(i) /
                           static_cast<double>(kNumSegments);
      points.push_back(
          make_map_path_point(kRadius * cos(p), kRadius * (sin(p) - 1.0)));
    } else {
      const double p = M_PI_2 -
                       2.0 * M_PI * static_cast<double>(i) /
                           static_cast<double>(kNumSegments);
      points.push_back(
          make_map_path_point(kRadius * cos(p), kRadius * (sin(p) + 1.0)));
    }
  }
  const double segment_length = points[0].DistanceTo(points[1]);
  std::vector<Lane> original_lanes;
  std::vector<LaneInfoConstPtr> lanes;
  for (int i = 0; i < kNumSegments; ++i) {
    Lane lane;
    lane.mutable_id()->set_id(to_string(i));
    auto* segment =
        lane.mutable_central_curve()->add_segment()->mutable_line_segment();
    auto* point1 = segment->add_point();
    point1->set_x(points[i].x());
    point1->set_y(points[i].y());
    auto* point2 = segment->add_point();
    point2->set_x(points[i + 1].x());
    point2->set_y(points[i + 1].y());
    original_lanes.push_back(lane);
  }
  for (int i = 0; i < kNumSegments; ++i) {
    lanes.push_back(LaneInfoConstPtr(new LaneInfo(original_lanes[i])));
  }
  for (int i = 0; i <= kNumSegments; ++i) {
    points[i].set_heading((i < kNumSegments)
                              ? (points[i + 1] - points[i]).Angle()
                              : (points[i] - points[i - 1]).Angle());
    if (i > 0) {
      points[i].add_lane_waypoint(LaneWaypoint(lanes[i - 1], segment_length));
    }
    if (i < kNumSegments) {
      points[i].add_lane_waypoint(LaneWaypoint(lanes[i], 0.0));
    }
  }
  const Path path(points);
  EXPECT_EQ(path.num_points(), kNumSegments + 1);
  EXPECT_EQ(path.num_segments(), kNumSegments);
  EXPECT_EQ(path.path_points().size(), kNumSegments + 1);
  EXPECT_EQ(path.segments().size(), kNumSegments);
  EXPECT_EQ(path.lane_segments_to_next_point().size(), kNumSegments);

  EXPECT_NEAR(path.length(), segment_length * kNumSegments, 1e-6);
  for (int i = 0; i <= kNumSegments; ++i) {
    MapPathPoint point = path.get_smooth_point(segment_length * i);
    EXPECT_NEAR(point.x(), points[i].x(), 1e-6);
    EXPECT_NEAR(point.y(), points[i].y(), 1e-6);
    EXPECT_NEAR(point.heading(), points[i].heading(), 1e-6);
    if (i == 0) {
      EXPECT_EQ(point.lane_waypoints().size(), 1);
      EXPECT_EQ(point.lane_waypoints()[0].lane->id().id(), to_string(i));
      EXPECT_NEAR(point.lane_waypoints()[0].s, 0.0, 1e-6);
    } else if (i == kNumSegments) {
      EXPECT_EQ(point.lane_waypoints().size(), 1);
      EXPECT_EQ(point.lane_waypoints()[0].lane->id().id(), to_string(i - 1));
      EXPECT_NEAR(point.lane_waypoints()[0].s, segment_length, 1e-6);
    } else {
      EXPECT_EQ(point.lane_waypoints().size(), 2);
      EXPECT_EQ(point.lane_waypoints()[0].lane->id().id(), to_string(i - 1));
      EXPECT_NEAR(point.lane_waypoints()[0].s, segment_length, 1e-6);
      EXPECT_EQ(point.lane_waypoints()[1].lane->id().id(), to_string(i));
      EXPECT_NEAR(point.lane_waypoints()[1].s, 0.0, 1e-6);
    }

    if (i < kNumSegments) {
      for (int case_id = 0; case_id < 20; ++case_id) {
        const double offset = random_double(0.01, 0.99) * segment_length;
        const double s = segment_length * i + offset;
        point = path.get_smooth_point(s);
        EXPECT_NEAR(point.x(),
                    points[i].x() + offset * cos(points[i].heading()), 1e-6);
        EXPECT_NEAR(point.y(),
                    points[i].y() + offset * sin(points[i].heading()), 1e-6);
        EXPECT_NEAR(point.heading(), points[i].heading(), 1e-6);
        EXPECT_EQ(point.lane_waypoints().size(), 1);
        EXPECT_EQ(point.lane_waypoints()[0].lane->id().id(), to_string(i));
        EXPECT_NEAR(point.lane_waypoints()[0].s, offset, 1e-6);
        const InterpolatedIndex index = path.get_index_from_s(s);
        EXPECT_EQ(index.id, i);
        EXPECT_NEAR(index.offset, offset, 1e-6);
        EXPECT_NEAR(path.get_s_from_index(index), s, 1e-6);
      }
    }
  }
  InterpolatedIndex index = path.get_index_from_s(0.0);
  EXPECT_EQ(index.id, 0);
  EXPECT_NEAR(index.offset, 0, 1e-6);
  EXPECT_NEAR(path.get_s_from_index(index), 0.0, 1e-6);
  index = path.get_index_from_s(-0.1);
  EXPECT_EQ(index.id, 0);
  EXPECT_NEAR(index.offset, 0, 1e-6);
  EXPECT_NEAR(path.get_s_from_index(index), 0.0, 1e-6);
  index = path.get_index_from_s(segment_length * kNumSegments);
  EXPECT_EQ(index.id, kNumSegments);
  EXPECT_NEAR(index.offset, 0, 1e-6);
  EXPECT_NEAR(path.get_s_from_index(index), segment_length * kNumSegments,
              1e-6);
  index = path.get_index_from_s(segment_length * kNumSegments + 0.2);
  EXPECT_EQ(index.id, kNumSegments);
  EXPECT_NEAR(index.offset, 0, 1e-6);
  EXPECT_NEAR(path.get_s_from_index(index), segment_length * kNumSegments,
              1e-6);
}

TEST(TestSuite, compute_lane_segments_from_points) {
  std::vector<MapPathPoint> points{make_map_path_point(2, 0),
                                   make_map_path_point(2, 1),
                                   make_map_path_point(2, 2)};
  Lane lane1;
  lane1.mutable_id()->set_id("id1");
  auto* curve = lane1.mutable_central_curve();
  auto* segment = curve->add_segment()->mutable_line_segment();
  *segment->add_point() = make_point(0, 0, 0);
  *segment->add_point() = make_point(1, 0, 0);
  // const LaneInfo lane_info1(lane1);
  LaneInfoConstPtr lane_info1(new LaneInfo(lane1));

  Lane lane2 = lane1;
  lane2.mutable_id()->set_id("id2");
  // const LaneInfo lane_info2(lane2);
  LaneInfoConstPtr lane_info2(new LaneInfo(lane2));

  points[0].add_lane_waypoint(LaneWaypoint(lane_info1, 0.1));
  points[1].add_lane_waypoint(LaneWaypoint(lane_info1, 0.7));
  points[1].add_lane_waypoint(LaneWaypoint(lane_info2, 0.0));
  points[2].add_lane_waypoint(LaneWaypoint(lane_info2, 0.4));
  const Path path(points);
  EXPECT_EQ(path.lane_segments().size(), 2);
  EXPECT_EQ(path.lane_segments()[0].lane->id().id(), "id1");
  EXPECT_NEAR(path.lane_segments()[0].start_s, 0.1, 1e-6);
  EXPECT_NEAR(path.lane_segments()[0].end_s, 0.7, 1e-6);
  EXPECT_EQ(path.lane_segments()[1].lane->id().id(), "id2");
  EXPECT_NEAR(path.lane_segments()[1].start_s, 0.0, 1e-6);
  EXPECT_NEAR(path.lane_segments()[1].end_s, 0.4, 1e-6);
}

}  // hdmap
}  // apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
