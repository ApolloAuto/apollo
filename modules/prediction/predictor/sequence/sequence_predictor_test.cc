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

#include "modules/prediction/predictor/sequence/sequence_predictor.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"

namespace apollo {
namespace prediction {

class SequencePredictorTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    std::string file =
        "modules/prediction/testdata/single_perception_vehicle_onlane.pb.txt";
    apollo::common::util::GetProtoFromFile(file, &perception_obstacles_);
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(SequencePredictorTest, General) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(0);
  EXPECT_EQ(perception_obstacle.id(), 1);
  MLPEvaluator mlp_evaluator;
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  mlp_evaluator.Evaluate(obstacle_ptr);
  SequencePredictor predictor;
  predictor.Predict(obstacle_ptr);
  EXPECT_EQ(predictor.NumOfTrajectories(), 0);
  LaneSequence* lane_seq = obstacle_ptr->mutable_latest_feature()
                                       ->mutable_lane()
                                       ->mutable_lane_graph()
                                       ->mutable_lane_sequence(0);
  std::string sequence_str = predictor.ToString(*lane_seq);
  EXPECT_GT(sequence_str.size(), 0);
  SequencePredictor::LaneChangeType lane_change_type =
      predictor.GetLaneChangeType(lane_seq->mutable_lane_segment(0)->lane_id(),
                                  *lane_seq);
  EXPECT_EQ(lane_change_type, SequencePredictor::LaneChangeType::STRAIGHT);

  EXPECT_TRUE(predictor.LaneSequenceWithMaxProb(lane_change_type, 0.5, 0.5));
  EXPECT_FALSE(predictor.LaneChangeWithMaxProb(lane_change_type, 0.5, 0.5));

  std::vector<bool> enable_lane_sequence(3, true);
  predictor.FilterLaneSequences(*obstacle_ptr->mutable_latest_feature(),
                                lane_seq->mutable_lane_segment(0)->lane_id(),
                                &enable_lane_sequence);
  EXPECT_TRUE(enable_lane_sequence[0]);

  predictor.Clear();
}

}  // namespace prediction
}  // namespace apollo
