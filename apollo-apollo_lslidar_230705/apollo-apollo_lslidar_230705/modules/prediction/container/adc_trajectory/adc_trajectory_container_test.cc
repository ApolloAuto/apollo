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

#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"

#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::math::Vec2d;
using ::apollo::hdmap::Id;
using ::apollo::planning::ADCTrajectory;

class ADCTrajectoryTest : public KMLMapBasedTest {
 public:
  ADCTrajectoryTest() : container_() {}

  virtual void SetUp() {
    PathPoint path_point;
    path_point.set_x(-455.182);
    path_point.set_y(-160.608);
    path_point.set_s(50.1);

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_.add_trajectory_point()->CopyFrom(trajectory_point);

    Id lane_id;
    lane_id.set_id("l164");
    trajectory_.add_lane_id()->CopyFrom(lane_id);
  }

 protected:
  ADCTrajectoryContainer container_;
  ADCTrajectory trajectory_;
};

TEST_F(ADCTrajectoryTest, InsertionWithProtection) {
  trajectory_.set_right_of_way_status(ADCTrajectory::PROTECTED);
  container_.Insert(trajectory_);
  Vec2d vec;
  vec.set_x(-455.182);
  vec.set_y(-160.608);
  container_.SetPosition(vec);
  EXPECT_TRUE(container_.IsProtected());

  PathPoint path_point;
  path_point.set_x(-438.537);
  path_point.set_y(-160.991);
  EXPECT_FALSE(container_.IsPointInJunction(path_point));
  EXPECT_EQ(container_.ADCJunction(), nullptr);

  LaneSequence non_overlap_lane_sequence;
  LaneSegment lane_segment;
  lane_segment.set_lane_id("l22");
  non_overlap_lane_sequence.add_lane_segment()->CopyFrom(lane_segment);
  EXPECT_FALSE(container_.HasOverlap(non_overlap_lane_sequence));

  LaneSequence overlap_lane_sequence;
  lane_segment.set_lane_id("l164");
  non_overlap_lane_sequence.add_lane_segment()->CopyFrom(lane_segment);
  EXPECT_TRUE(container_.HasOverlap(non_overlap_lane_sequence));
}

TEST_F(ADCTrajectoryTest, InsertionWithoutProtection) {
  trajectory_.set_right_of_way_status(ADCTrajectory::UNPROTECTED);
  container_.Insert(trajectory_);
  EXPECT_FALSE(container_.IsProtected());
}

}  // namespace prediction
}  // namespace apollo
