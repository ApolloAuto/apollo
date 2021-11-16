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

#include "modules/prediction/common/prediction_map.h"

#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;
using apollo::hdmap::MapPathPoint;

class PredictionMapTest : public KMLMapBasedTest {};

TEST_F(PredictionMapTest, get_lane_info) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");
  EXPECT_NE(nullptr, lane_info);
  EXPECT_EQ("l20", lane_info->id().id());

  lane_info = PredictionMap::LaneById("l500");
  EXPECT_EQ(nullptr, lane_info);
}

TEST_F(PredictionMapTest, get_position_on_lane) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");

  // on lane
  Eigen::Vector2d position_on_lane =
      PredictionMap::PositionOnLane(lane_info, 10.0);
  EXPECT_DOUBLE_EQ(124.85930930657942, position_on_lane(0));
  EXPECT_DOUBLE_EQ(348.52732962417451, position_on_lane(1));

  // beyond end of lane
  Eigen::Vector2d position_off_lane =
      PredictionMap::PositionOnLane(lane_info, 1000.0);
  EXPECT_DOUBLE_EQ(392.71861332684404, position_off_lane(0));
  EXPECT_DOUBLE_EQ(286.16205764480401, position_off_lane(1));
}

TEST_F(PredictionMapTest, heading_on_lane) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");

  // on lane
  EXPECT_DOUBLE_EQ(-0.066794953844859783,
                   PredictionMap::HeadingOnLane(lane_info, 10.0));
}

TEST_F(PredictionMapTest, get_lane_width) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");

  // on lane
  EXPECT_DOUBLE_EQ(2.9895597224833121,
                   PredictionMap::LaneTotalWidth(lane_info, 10.0));

  // beyond end of lane
  EXPECT_DOUBLE_EQ(3.1943980708125523,
                   PredictionMap::LaneTotalWidth(lane_info, 1000.0));
  EXPECT_DOUBLE_EQ(3.1943980708125523,
                   PredictionMap::LaneTotalWidth(lane_info, 1000.0));
}

TEST_F(PredictionMapTest, get_projection) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");

  // on lane
  Eigen::Vector2d position_on_lane(124.85931, 347.52733);
  double s = 0.0;
  double l = 0.0;
  PredictionMap::GetProjection(position_on_lane, lane_info, &s, &l);
  EXPECT_DOUBLE_EQ(10.061275933723756, s);
  EXPECT_DOUBLE_EQ(-0.9981204878650296, l);

  // off lane
  Eigen::Vector2d position_off_lane(124.85931, 357.52733);
  PredictionMap::GetProjection(position_off_lane, lane_info, &s, &l);
  EXPECT_DOUBLE_EQ(9.4485232873738045, s);
  EXPECT_DOUBLE_EQ(8.9830885668733345, l);
}

TEST_F(PredictionMapTest, get_map_pathpoint) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");
  double s = 10.0;

  // on lane
  MapPathPoint point;
  EXPECT_TRUE(PredictionMap::ProjectionFromLane(lane_info, s, &point));
  EXPECT_DOUBLE_EQ(124.85930930657942, point.x());
  EXPECT_DOUBLE_EQ(348.52732962417451, point.y());
  EXPECT_DOUBLE_EQ(-0.066794953844859783, point.heading());

  // non-existent lane
  lane_info.reset();
  s = 10.0;
  EXPECT_FALSE(PredictionMap::ProjectionFromLane(lane_info, s, &point));
}

TEST_F(PredictionMapTest, on_lane) {
  std::vector<std::shared_ptr<const LaneInfo>> prev_lanes(0);
  Eigen::Vector2d point(124.85931, 347.52733);
  double heading = 0.0;
  double radius = 3.0;

  // on lane without previous lanes
  std::vector<std::shared_ptr<const LaneInfo>> curr_lanes(0);
  PredictionMap::OnLane(prev_lanes, point, heading, radius, true,
                        FLAGS_max_num_current_lane, FLAGS_max_lane_angle_diff,
                        &curr_lanes);
  EXPECT_EQ(1, curr_lanes.size());
  EXPECT_EQ("l20", curr_lanes[0]->id().id());

  // on lane with previous lanes
  prev_lanes.emplace_back(PredictionMap::LaneById("l10"));
  curr_lanes.clear();
  PredictionMap::OnLane(prev_lanes, point, heading, radius, true,
                        FLAGS_max_num_current_lane, FLAGS_max_lane_angle_diff,
                        &curr_lanes);
  EXPECT_EQ(0, curr_lanes.size());

  // off lane without previous lanes
  prev_lanes.clear();
  point(0) = 124.85931;
  point(1) = 357.52733;
  curr_lanes.clear();
  PredictionMap::OnLane(prev_lanes, point, heading, radius, true,
                        FLAGS_max_num_current_lane, FLAGS_max_lane_angle_diff,
                        &curr_lanes);
  EXPECT_TRUE(curr_lanes.empty());
}

TEST_F(PredictionMapTest, get_path_heading) {
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById("l20");
  common::PointENU point;
  point.set_x(124.85931);
  point.set_y(347.52733);
  auto actual_heading = PredictionMap::PathHeading(lane_info, point);
  EXPECT_NEAR(-0.066973788088279029, actual_heading, 1.0e-10);
}

TEST_F(PredictionMapTest, get_smooth_point_from_lane) {
  const std::string id = "l20";
  double s = 10.0;
  double l = 0.0;
  double heading = M_PI;

  Eigen::Vector2d point;
  EXPECT_TRUE(PredictionMap::SmoothPointFromLane(id, s, l, &point, &heading));
  EXPECT_DOUBLE_EQ(124.85930930657942, point.x());
  EXPECT_DOUBLE_EQ(348.52732962417451, point.y());
  EXPECT_DOUBLE_EQ(-0.066794953844859783, heading);
}

TEST_F(PredictionMapTest, get_nearby_lanes_by_current_lanes) {
  std::vector<std::shared_ptr<const LaneInfo>> curr_lanes(0);
  curr_lanes.emplace_back(PredictionMap::LaneById("l20"));
  std::vector<std::shared_ptr<const LaneInfo>> nearby_lanes(0);

  // large radius
  Eigen::Vector2d point(124.85931, 348.52733);
  double radius = 6.0;
  double theta = -0.061427808505166936;
  PredictionMap::NearbyLanesByCurrentLanes(point, theta, radius, curr_lanes,
                                           FLAGS_max_num_nearby_lane,
                                           &nearby_lanes);
  EXPECT_EQ(1, nearby_lanes.size());
  EXPECT_EQ("l21", nearby_lanes[0]->id().id());

  // small radius
  nearby_lanes.clear();
  radius = 0.5;
  PredictionMap::NearbyLanesByCurrentLanes(point, theta, radius, curr_lanes,
                                           FLAGS_max_num_nearby_lane,
                                           &nearby_lanes);
  EXPECT_EQ(0, nearby_lanes.size());

  // without current lanes
  curr_lanes.clear();
  nearby_lanes.clear();
  radius = 5.0;
  PredictionMap::NearbyLanesByCurrentLanes(point, theta, radius, curr_lanes,
                                           FLAGS_max_num_nearby_lane,
                                           &nearby_lanes);
  EXPECT_EQ(2, nearby_lanes.size());
  EXPECT_EQ("l20", nearby_lanes[0]->id().id());
  EXPECT_EQ("l21", nearby_lanes[1]->id().id());
}

TEST_F(PredictionMapTest, neighbor_lane_detection) {
  std::vector<std::shared_ptr<const LaneInfo>> curr_lanes(0);

  // empty current lanes
  EXPECT_TRUE(PredictionMap::IsLeftNeighborLane(PredictionMap::LaneById("l20"),
                                                curr_lanes));
  EXPECT_TRUE(PredictionMap::IsRightNeighborLane(PredictionMap::LaneById("l20"),
                                                 curr_lanes));
  EXPECT_TRUE(PredictionMap::IsSuccessorLane(PredictionMap::LaneById("l20"),
                                             curr_lanes));
  EXPECT_TRUE(PredictionMap::IsPredecessorLane(PredictionMap::LaneById("l20"),
                                               curr_lanes));
  EXPECT_TRUE(PredictionMap::IsIdenticalLane(PredictionMap::LaneById("l20"),
                                             curr_lanes));

  // given current lanes
  std::shared_ptr<const LaneInfo> curr_lane = PredictionMap::LaneById("l21");
  curr_lanes.emplace_back(curr_lane);
  EXPECT_TRUE(PredictionMap::IsLeftNeighborLane(PredictionMap::LaneById("l22"),
                                                curr_lanes));
  EXPECT_FALSE(PredictionMap::IsRightNeighborLane(
      PredictionMap::LaneById("l22"), curr_lanes));
  EXPECT_FALSE(PredictionMap::IsSuccessorLane(PredictionMap::LaneById("l22"),
                                              curr_lanes));
  EXPECT_FALSE(PredictionMap::IsPredecessorLane(PredictionMap::LaneById("l22"),
                                                curr_lanes));
  EXPECT_FALSE(PredictionMap::IsIdenticalLane(PredictionMap::LaneById("l22"),
                                              curr_lanes));

  EXPECT_FALSE(PredictionMap::IsLeftNeighborLane(PredictionMap::LaneById("l20"),
                                                 curr_lanes));
  EXPECT_TRUE(PredictionMap::IsRightNeighborLane(PredictionMap::LaneById("l20"),
                                                 curr_lanes));
  EXPECT_FALSE(PredictionMap::IsSuccessorLane(PredictionMap::LaneById("l20"),
                                              curr_lanes));
  EXPECT_FALSE(PredictionMap::IsPredecessorLane(PredictionMap::LaneById("l20"),
                                                curr_lanes));
  EXPECT_FALSE(PredictionMap::IsIdenticalLane(PredictionMap::LaneById("l20"),
                                              curr_lanes));

  EXPECT_FALSE(PredictionMap::IsLeftNeighborLane(PredictionMap::LaneById("l18"),
                                                 curr_lanes));
  EXPECT_FALSE(PredictionMap::IsRightNeighborLane(
      PredictionMap::LaneById("l18"), curr_lanes));
  EXPECT_FALSE(PredictionMap::IsSuccessorLane(PredictionMap::LaneById("l18"),
                                              curr_lanes));
  EXPECT_TRUE(PredictionMap::IsPredecessorLane(PredictionMap::LaneById("l18"),
                                               curr_lanes));
  EXPECT_FALSE(PredictionMap::IsIdenticalLane(PredictionMap::LaneById("l18"),
                                              curr_lanes));

  EXPECT_FALSE(PredictionMap::IsLeftNeighborLane(PredictionMap::LaneById("l99"),
                                                 curr_lanes));
  EXPECT_FALSE(PredictionMap::IsRightNeighborLane(
      PredictionMap::LaneById("l99"), curr_lanes));
  EXPECT_TRUE(PredictionMap::IsSuccessorLane(PredictionMap::LaneById("l99"),
                                             curr_lanes));
  EXPECT_FALSE(PredictionMap::IsPredecessorLane(PredictionMap::LaneById("l99"),
                                                curr_lanes));
  EXPECT_FALSE(PredictionMap::IsIdenticalLane(PredictionMap::LaneById("l99"),
                                              curr_lanes));
}

TEST_F(PredictionMapTest, lane_turn_type) {
  // Valid lane
  EXPECT_EQ(1, PredictionMap::LaneTurnType("l20"));

  // Invalid lane
  EXPECT_FALSE(PredictionMap::LaneById("l500"));
  EXPECT_EQ(1, PredictionMap::LaneTurnType("l500"));

  EXPECT_EQ(3, PredictionMap::LaneTurnType("l5"));
}

}  // namespace prediction
}  // namespace apollo
