/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/container/obstacles/obstacle_clusters.h"

#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

class ObstacleClustersTest : public KMLMapBasedTest {};

TEST_F(ObstacleClustersTest, ObstacleClusters) {
  auto lane = PredictionMap::LaneById("l9");
  double start_s = 99.0;
  double length = 100.0;

  ObstacleClusters cluster;
  const LaneGraph &lane_graph =
      cluster.GetLaneGraph(start_s, length, true, lane);
  EXPECT_EQ(1, lane_graph.lane_sequence_size());
  EXPECT_EQ(3, lane_graph.lane_sequence(0).lane_segment_size());
  EXPECT_EQ("l9", lane_graph.lane_sequence(0).lane_segment(0).lane_id());
  EXPECT_EQ("l18", lane_graph.lane_sequence(0).lane_segment(1).lane_id());
  EXPECT_EQ("l21", lane_graph.lane_sequence(0).lane_segment(2).lane_id());

  double length_2 = 50.0;
  const LaneGraph &lane_graph_2 =
      cluster.GetLaneGraph(start_s, length_2, true, lane);
  EXPECT_EQ(1, lane_graph_2.lane_sequence_size());
  EXPECT_EQ(2, lane_graph_2.lane_sequence(0).lane_segment_size());
  EXPECT_EQ("l9", lane_graph_2.lane_sequence(0).lane_segment(0).lane_id());
  EXPECT_EQ("l18", lane_graph_2.lane_sequence(0).lane_segment(1).lane_id());
}

}  // namespace prediction
}  // namespace apollo
