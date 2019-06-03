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

#include "modules/prediction/common/road_graph.h"

#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

class RoadGraphTest : public KMLMapBasedTest {};

TEST_F(RoadGraphTest, General) {
  auto lane = PredictionMap::LaneById("l9");
  EXPECT_NE(lane, nullptr);

  double start_s = 99.0;
  double length = 100.0;
  RoadGraph road_graph(start_s, length, true, lane);

  LaneGraph lane_graph;
  EXPECT_TRUE(road_graph.BuildLaneGraph(&lane_graph).ok());
  EXPECT_EQ(1, lane_graph.lane_sequence_size());
  EXPECT_EQ(3, lane_graph.lane_sequence(0).lane_segment_size());
  EXPECT_EQ("l9", lane_graph.lane_sequence(0).lane_segment(0).lane_id());
  EXPECT_EQ("l18", lane_graph.lane_sequence(0).lane_segment(1).lane_id());
  EXPECT_EQ("l21", lane_graph.lane_sequence(0).lane_segment(2).lane_id());

  EXPECT_TRUE(
      road_graph.IsOnLaneGraph(PredictionMap::LaneById("l9"), lane_graph));
  EXPECT_TRUE(
      road_graph.IsOnLaneGraph(PredictionMap::LaneById("l18"), lane_graph));
  EXPECT_TRUE(
      road_graph.IsOnLaneGraph(PredictionMap::LaneById("l21"), lane_graph));
  EXPECT_FALSE(
      road_graph.IsOnLaneGraph(PredictionMap::LaneById("l30"), lane_graph));

  for (const auto &lane_sequence : lane_graph.lane_sequence()) {
    double total_length = 0.0;
    for (const auto &lane_segment : lane_sequence.lane_segment()) {
      total_length += (lane_segment.end_s() - lane_segment.start_s());
    }
    EXPECT_DOUBLE_EQ(length, total_length);
  }
}

/*
TEST_F(RoadGraphTest, NegativeStartS) {
  auto lane = PredictionMap::LaneById("l9");
  EXPECT_NE(lane, nullptr);

  double start_s = -10.0;
  double length = 50.0;
  RoadGraph road_graph(start_s, length, true, lane);

  LaneGraph lane_graph;
  EXPECT_TRUE(road_graph.BuildLaneGraph(&lane_graph).ok());
  EXPECT_EQ(1, lane_graph.lane_sequence_size());
  EXPECT_EQ(1, lane_graph.lane_sequence(0).lane_segment_size());
  EXPECT_EQ("l9", lane_graph.lane_sequence(0).lane_segment(0).lane_id());

  for (const auto &lane_sequence : lane_graph.lane_sequence()) {
    double total_length = 0.0;
    for (const auto &lane_segment : lane_sequence.lane_segment()) {
      total_length += (lane_segment.end_s() - lane_segment.start_s());
    }
    EXPECT_DOUBLE_EQ(length, total_length);
  }
}
*/

TEST_F(RoadGraphTest, LengthLongerThanEnd) {
  auto lane = PredictionMap::LaneById("l22");
  EXPECT_NE(lane, nullptr);

  double start_s = 200.0;
  double length = 200.0;
  RoadGraph road_graph(start_s, length, true, lane);

  LaneGraph lane_graph;
  EXPECT_TRUE(road_graph.BuildLaneGraph(&lane_graph).ok());
  EXPECT_EQ(1, lane_graph.lane_sequence_size());
  EXPECT_EQ(3, lane_graph.lane_sequence(0).lane_segment_size());
  EXPECT_EQ("l22", lane_graph.lane_sequence(0).lane_segment(0).lane_id());
  EXPECT_EQ("l100", lane_graph.lane_sequence(0).lane_segment(1).lane_id());
  EXPECT_EQ("l97", lane_graph.lane_sequence(0).lane_segment(2).lane_id());

  for (const auto &lane_sequence : lane_graph.lane_sequence()) {
    double total_length = 0.0;
    for (const auto &lane_segment : lane_sequence.lane_segment()) {
      total_length += (lane_segment.end_s() - lane_segment.start_s());
    }
    EXPECT_LT(total_length, length);
  }
}

TEST_F(RoadGraphTest, MultipleLaneSequence) {
  auto lane = PredictionMap::LaneById("l20");
  EXPECT_NE(lane, nullptr);

  double start_s = 200.0;
  double length = 200.0;
  RoadGraph road_graph(start_s, length, true, lane);

  LaneGraph lane_graph;
  EXPECT_TRUE(road_graph.BuildLaneGraph(&lane_graph).ok());
  EXPECT_EQ(2, lane_graph.lane_sequence_size());
  EXPECT_EQ(3, lane_graph.lane_sequence(0).lane_segment_size());
  EXPECT_EQ(3, lane_graph.lane_sequence(1).lane_segment_size());
  EXPECT_EQ("l20", lane_graph.lane_sequence(0).lane_segment(0).lane_id());
  EXPECT_EQ("l98", lane_graph.lane_sequence(0).lane_segment(1).lane_id());
  EXPECT_EQ("l95", lane_graph.lane_sequence(0).lane_segment(2).lane_id());
  EXPECT_EQ("l20", lane_graph.lane_sequence(1).lane_segment(0).lane_id());
  EXPECT_EQ("l31", lane_graph.lane_sequence(1).lane_segment(1).lane_id());
  EXPECT_EQ("l29", lane_graph.lane_sequence(1).lane_segment(2).lane_id());
}

}  // namespace prediction
}  // namespace apollo
