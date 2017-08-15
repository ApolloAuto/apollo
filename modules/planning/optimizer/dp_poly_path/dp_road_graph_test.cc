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

#include <algorithm>
#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/optimizer/dp_poly_path/dp_road_graph.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

class DpRoadGraphTest : public PlanningTestBase {
 public:
  void SetInitPoint() {
    const auto* frame = planning_.GetFrame();
    const auto& pose = frame->VehicleInitPose();
    init_point_.mutable_path_point()->set_x(pose.position().x());
    init_point_.mutable_path_point()->set_y(pose.position().y());
    const auto& velocity = pose.linear_velocity();
    init_point_.set_v(std::hypot(velocity.x(), velocity.y()));
    const auto& acc = pose.linear_acceleration();
    init_point_.set_a(std::hypot(acc.x(), acc.y()));
    init_point_.set_relative_time(0.0);
  }

  void SetSpeedDataWithConstVeolocity(const double velocity) {
    // speed point params: s, time, v, a, da
    double t = 0.0;
    const double delta_s = 1.0;
    if (velocity > 0.0) {
      for (double s = 0.0; s < 100.0; s += delta_s) {
        speed_data_.AppendSpeedPoint(s, t, velocity, 0.0, 0.0);
        t += delta_s / velocity;
      }
    } else {  // when velocity = 0, stand still
      for (double t = 0.0; t < 10.0; t += 0.1) {
        speed_data_.AppendSpeedPoint(0.0, t, 0.0, 0.0, 0.0);
      }
    }
  }

  virtual void SetUp() {
    google::InitGoogleLogging("DpRoadGraphTest");
    PlanningTestBase::SetUp();
    RunPlanning();
    SetInitPoint();
    SetSpeedDataWithConstVeolocity(10.0);
    const auto* frame = planning_.GetFrame();
    ASSERT_TRUE(frame != nullptr);
    reference_line_ = &(frame->reference_line());
  }

 protected:
  const ReferenceLine* reference_line_ = nullptr;
  common::TrajectoryPoint init_point_;
  SpeedData speed_data_;  // input
  PathData path_data_;    // output
};

TEST_F(DpRoadGraphTest, speed_road_graph) {
  DpPolyPathConfig dp_poly_path_config;  // default values
  DPRoadGraph road_graph(dp_poly_path_config, *reference_line_, speed_data_);
  ASSERT_TRUE(reference_line_ != nullptr);
  bool result = road_graph.FindPathTunnel(init_point_, &path_data_);
  EXPECT_TRUE(result);
  EXPECT_EQ(648, path_data_.discretized_path().NumOfPoints());
  EXPECT_EQ(648, path_data_.frenet_frame_path().NumOfPoints());
  EXPECT_FLOAT_EQ(70.450378,
                  path_data_.frenet_frame_path().points().back().s());
  EXPECT_FLOAT_EQ(0.0, path_data_.frenet_frame_path().points().back().l());
  // export_path_data(path_data_, "/tmp/path.txt");
}

}  // namespace planning
}  // namespace apollo
