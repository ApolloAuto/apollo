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

/**
 * @file
 **/

#include <memory>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#include "Eigen/Eigen"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/frame_open_space.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::perception::PerceptionObstacle;

class FrameTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    ASSERT_TRUE(common::util::GetProtoFromFile(
        "modules/planning/common/testdata/sample_prediction.pb.txt",
        &prediction_obstacles_));
    sequence_num_ = 1;
    start_time_ = 1535757683;
    vehicle_state_.set_x(587419.24);
    vehicle_state_.set_y(4141269.79);
    test_frame_ = new FrameOpenSpace(sequence_num_, planning_start_point_,
                                     start_time_, vehicle_state_);
  }

 protected:
  prediction::PredictionObstacles prediction_obstacles_;
  FrameOpenSpace* test_frame_;
  uint32_t sequence_num_;
  apollo::common::TrajectoryPoint planning_start_point_;
  double start_time_;
  apollo::common::VehicleState vehicle_state_;
};

TEST_F(FrameTest, AlignPredictionTime) {
  int first_traj_size = prediction_obstacles_.prediction_obstacle(0)
                            .trajectory(0)
                            .trajectory_point_size();
  double origin_pred_time = prediction_obstacles_.header().timestamp_sec();
  FrameOpenSpace::AlignPredictionTime(origin_pred_time + 0.1,
                                      &prediction_obstacles_);
  ASSERT_EQ(first_traj_size - 1, prediction_obstacles_.prediction_obstacle(0)
                                     .trajectory(0)
                                     .trajectory_point_size());

  FrameOpenSpace::AlignPredictionTime(origin_pred_time + 0.5,
                                      &prediction_obstacles_);
  ASSERT_EQ(first_traj_size - 3, prediction_obstacles_.prediction_obstacle(0)
                                     .trajectory(0)
                                     .trajectory_point_size());

  FrameOpenSpace::AlignPredictionTime(origin_pred_time + 12.0,
                                      &prediction_obstacles_);
  ASSERT_EQ(0, prediction_obstacles_.prediction_obstacle(0)
                   .trajectory(0)
                   .trajectory_point_size());
}

TEST_F(FrameTest, is_near_destination) {
  ASSERT_FALSE(test_frame_->is_near_destination());
}

TEST_F(FrameTest, vehicle_state) {
  apollo::common::VehicleState vehicle_state_test_;
  vehicle_state_test_.set_x(587419.24);
  vehicle_state_test_.set_y(4141269.79);
  apollo::common::VehicleState vehicle_state_result =
      test_frame_->vehicle_state();
  double x = vehicle_state_result.x();
  double y = vehicle_state_result.y();
  ASSERT_EQ(vehicle_state_test_.x(), x);
  ASSERT_EQ(vehicle_state_test_.y(), y);
}

TEST_F(FrameTest, sequence_num) {
  double sequence_num_test = 1;
  ASSERT_EQ(test_frame_->SequenceNum(), sequence_num_test);
}

TEST_F(FrameTest, Hpresentation_Obstacle) {
  test_frame_->HPresentationObstacle();
}

TEST_F(FrameTest, obstacle_H_presentation) {
  std::size_t obstacles_num = 2;
  Eigen::MatrixXd obstacles_vertices_num(obstacles_num, 1);
  obstacles_vertices_num << 4, 4;
  int edges_num = static_cast<int>(obstacles_vertices_num.sum());
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec = {
      {Vec2d(0, 0), Vec2d(-1, 1), Vec2d(0, 2), Vec2d(1, 1), Vec2d(0, 0)},
      {Vec2d(0, -1), Vec2d(2, -1), Vec2d(2, -2), Vec2d(0, -2), Vec2d(0, -1)}};
  Eigen::MatrixXd expect_A = Eigen::MatrixXd::Zero(edges_num, 2);
  Eigen::MatrixXd expect_b = Eigen::MatrixXd::Zero(edges_num, 1);
  ASSERT_TRUE(test_frame_->ObsHRep(obstacles_num, obstacles_vertices_num,
                                   obstacles_vertices_vec, &expect_A,
                                   &expect_b));
  Eigen::MatrixXd actual_A(edges_num, 2);
  Eigen::MatrixXd actual_b(edges_num, 1);
  actual_A << -1, -1, -1, 1, 1, 1, 1, -1, 0, 1, 1, 0, 0, -1, -1, 0;
  actual_b << 0, 2, 2, 0, -1, 2, 2, 0;
  ASSERT_LT((actual_A - expect_A).norm(), 1e-5);
  ASSERT_LT((actual_b - expect_b).norm(), 1e-5);
}

}  // namespace planning
}  // namespace apollo
