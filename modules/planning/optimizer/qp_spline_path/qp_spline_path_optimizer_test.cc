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

#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/common/util/file.h"
#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_generator.h"
#include "modules/common/vehicle_state/vehicle_state.h"

namespace apollo {
namespace planning {

using PlanningPb = planning::ADCTrajectory;

class QpSplinePathOptimizerTest : public ::testing::Test {
  virtual void SetUp() {
    FLAGS_reference_line_smoother_config_file =
        "modules/planning/testdata/conf/reference_line_smoother_config.pb.txt";
    FLAGS_planning_config_file =
        "modules/planning/testdata/qp_spline_path/planning_config.pb.txt";
    optimizer_.reset(new QpSplinePathOptimizer(""));
    frame.reset(new Frame(1));
    hdmap::PncMap* pnc_map;
    pnc_map = new hdmap::PncMap("modules/planning/testdata/base_map.txt");
    Frame::SetMap(pnc_map);

    LoadPlanningConfig();
    planning_pb = LoadPlanningPb(
        "modules/planning/testdata"
            "/qp_spline_path/42_apollo_planning.pb.txt");
    frame->SetVehicleInitPose(
        planning_pb.debug().planning_data().adc_position().pose());
    frame->SetRoutingResponse(
        planning_pb.debug().planning_data().routing());
  }

 protected:
  PlanningPb LoadPlanningPb(const std::string &filename) {
    PlanningPb planning_pb;
    CHECK(apollo::common::util::GetProtoFromFile(filename, &planning_pb))
    << "Failed to open file " << filename;
    return std::move(planning_pb);
  }

  void LoadPlanningConfig() {
    PlanningPb planning_pb;
    CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                                 &planning_config_))
    << "Failed to open file " << FLAGS_planning_config_file;
  }

  common::TrajectoryPoint GetInitPoint() {
    common::TrajectoryPoint init_point;
    common::VehicleState::instance()->Update(
        planning_pb.debug().planning_data().adc_position(),
        planning_pb.debug().planning_data().chassis());
    init_point.mutable_path_point()->set_x(
        common::VehicleState::instance()->x());
    init_point.mutable_path_point()->set_y(
        common::VehicleState::instance()->y());
    init_point.set_v(common::VehicleState::instance()->linear_velocity());
    init_point.set_a(common::VehicleState::instance()->linear_acceleration());
    init_point.mutable_path_point()->set_theta(
        common::VehicleState::instance()->heading());
    init_point.mutable_path_point()->set_kappa(
        common::VehicleState::instance()->kappa());

    init_point.set_relative_time(0.0);
    return std::move(init_point);
  }

  double timestamp_ = 0.0;
  std::unique_ptr<Frame> frame;
  PlanningPb planning_pb;
  std::unique_ptr<Optimizer> optimizer_;
  PlanningConfig planning_config_;
};

TEST_F(QpSplinePathOptimizerTest, Process) {
  EXPECT_EQ(frame->Init(), true);
  common::TrajectoryPoint init_point = GetInitPoint();
  frame->SetPlanningStartPoint(init_point);
  PlanningData* planning_data = frame->mutable_planning_data();
  EXPECT_EQ(planning_data->path_data().discretized_path().path_points().size()
  , 0);
  optimizer_->Init(planning_config_);
  optimizer_->Optimize(frame.get());

  common::Path qp_path_ground_truth;
  for (common::Path path : planning_pb.debug().planning_data().path()) {
    if (path.name() == "QP_SPLINE_PATH_OPTIMIZER") {
      qp_path_ground_truth = path;
    }
  }

  EXPECT_EQ(qp_path_ground_truth.path_point().size(), 101);
  EXPECT_EQ(frame->planning_data().path_data()
                .discretized_path().path_points().size(), 101);

  for (std::int32_t i = 0; i < qp_path_ground_truth.path_point().size(); i++) {
    common::PathPoint ground_truth_point =
        qp_path_ground_truth.path_point().Get(i);
    common::PathPoint computed_point = planning_data
        ->path_data().discretized_path().path_points()[i];
    EXPECT_NEAR(ground_truth_point.x(), computed_point.x(), 1.0e-2);
    EXPECT_NEAR(ground_truth_point.y(), computed_point.y(), 1.0e-3);
    EXPECT_NEAR(ground_truth_point.kappa(), computed_point.kappa(), 1.0e-3);
    EXPECT_NEAR(ground_truth_point.theta(), computed_point.theta(), 1.0e-3);
    EXPECT_NEAR(ground_truth_point.s(), computed_point.s(), 1.0e-3);
    EXPECT_NEAR(ground_truth_point.dkappa(), computed_point.dkappa(), 1.0e-3);
    EXPECT_NEAR(ground_truth_point.ddkappa(), computed_point.ddkappa(), 1.0e-3);
  }
}

}  // namespace planning
}  // namespace apollo
