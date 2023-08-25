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

#include "modules/control/control_component/controller_task_base/common/trajectory_analyzer.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;

namespace apollo {
namespace control {

class TrajectoryAnalyzerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

void SetTrajectory(const std::vector<double> &xs, const std::vector<double> &ys,
                   planning::ADCTrajectory *adc_trajectory) {
  for (size_t i = 0; i < xs.size(); ++i) {
    auto *point = adc_trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(xs[i]);
    point->mutable_path_point()->set_y(ys[i]);
  }
  adc_trajectory->mutable_header()->set_sequence_num(123);
}

void SetTrajectoryWithTime(const std::vector<double> &xs,
                           const std::vector<double> &ys,
                           const std::vector<double> &ts,
                           planning::ADCTrajectory *adc_trajectory) {
  for (size_t i = 0; i < xs.size(); ++i) {
    auto *point = adc_trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(xs[i]);
    point->mutable_path_point()->set_y(ys[i]);
    point->set_relative_time(ts[i]);
  }
}

void SetTrajectory(const std::vector<double> &xs, const std::vector<double> &ys,
                   const std::vector<double> &ss,
                   planning::ADCTrajectory *adc_trajectory) {
  for (size_t i = 0; i < xs.size(); ++i) {
    auto *point = adc_trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(xs[i]);
    point->mutable_path_point()->set_y(ys[i]);
    point->mutable_path_point()->set_s(ss[i]);
  }
}

TEST_F(TrajectoryAnalyzerTest, SetADCTrajectory) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
  std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
  SetTrajectory(xs, ys, &adc_trajectory);
  int traj_size = adc_trajectory.trajectory_point_size();
  EXPECT_EQ(traj_size, 5);
  for (int i = 0; i < traj_size; ++i) {
    EXPECT_EQ(adc_trajectory.trajectory_point(i).path_point().x(), xs[i]);
    EXPECT_EQ(adc_trajectory.trajectory_point(i).path_point().y(), ys[i]);
  }
}

TEST_F(TrajectoryAnalyzerTest, Constructor) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
  std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
  SetTrajectory(xs, ys, &adc_trajectory);

  TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);
  EXPECT_EQ(trajectory_analyzer.trajectory_points().size(), 5);
  int i = 0;
  for (auto &point : trajectory_analyzer.trajectory_points()) {
    EXPECT_EQ(xs[i], point.path_point().x());
    EXPECT_EQ(ys[i], point.path_point().y());
    ++i;
  }
  EXPECT_EQ(trajectory_analyzer.seq_num(), 123);
}

TEST_F(TrajectoryAnalyzerTest, QueryMatchedPathPoint) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.8};
  std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.8};
  std::vector<double> ss = {1.0, 1.1, 1.2, 1.3, 1.8};
  SetTrajectory(xs, ys, ss, &adc_trajectory);
  TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);

  PathPoint point = trajectory_analyzer.QueryMatchedPathPoint(1.21, 1.21);
  EXPECT_NEAR(point.x(), 1.21, 1e-3);
  EXPECT_NEAR(point.y(), 1.21, 1e-3);

  point = trajectory_analyzer.QueryMatchedPathPoint(1.56, 1.50);
  EXPECT_NEAR(point.x(), 1.53, 1e-3);
  EXPECT_NEAR(point.y(), 1.53, 1e-3);
}

TEST_F(TrajectoryAnalyzerTest, ToTrajectoryFrame) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs = {0.8, 0.9, 1.0, 1.1, 1.2};
  std::vector<double> ys = {0.0, 0.0, 0.0, 0.0, 0.0};
  SetTrajectory(xs, ys, &adc_trajectory);

  TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);
  double x = 2.0;
  double y = 1.0;
  double theta = M_PI / 4.0;
  double v = 1.0;
  PathPoint ref_point;
  ref_point.set_x(1.0);
  ref_point.set_y(0.0);
  ref_point.set_theta(0.0);
  ref_point.set_s(1.0);

  double s = 0.0;
  double s_dot = 0.0;
  double d = 0.0;
  double d_dot = 0.0;
  trajectory_analyzer.ToTrajectoryFrame(x, y, theta, v, ref_point, &s, &s_dot,
                                        &d, &d_dot);
  EXPECT_NEAR(s, 2.0, 1e-6);
  EXPECT_NEAR(s_dot, 0.707, 1e-3);
  EXPECT_NEAR(d, 1.0, 1e-3);
  EXPECT_NEAR(d_dot, 0.707, 1e-3);
}

TEST_F(TrajectoryAnalyzerTest, QueryNearestPointByAbsoluteTimeInterpolation) {
  planning::ADCTrajectory adc_trajectory;
  std::vector<double> xs = {1.0, 1.1, 1.2, 1.3, 1.4};
  std::vector<double> ys = {1.0, 1.1, 1.2, 1.3, 1.4};
  std::vector<double> ts = {0.0, 0.1, 0.2, 0.3, 0.4};
  SetTrajectoryWithTime(xs, ys, ts, &adc_trajectory);

  double timestamp_ = Clock::NowInSeconds() - 2.0;
  adc_trajectory.mutable_header()->set_timestamp_sec(timestamp_);
  TrajectoryAnalyzer trajectory_analyzer(&adc_trajectory);

  double current_time = Clock::NowInSeconds() - 20.0;
  TrajectoryPoint point_2 =
      trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_time);
  EXPECT_NEAR(point_2.path_point().x(), 1.0, 1e-6);

  current_time = timestamp_ + 50.0;
  TrajectoryPoint point_4 =
      trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_time);
  EXPECT_NEAR(point_4.path_point().x(), 1.4, 1e-6);

  current_time = timestamp_ + 0.03;
  TrajectoryPoint point_6 =
      trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_time);
  EXPECT_NEAR(point_6.path_point().x(), 1.0, 1e-6);
}

}  // namespace control
}  // namespace apollo
