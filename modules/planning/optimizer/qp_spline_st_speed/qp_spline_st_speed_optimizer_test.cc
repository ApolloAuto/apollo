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
#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"

namespace apollo {
namespace planning {

using PlanningPb = planning::ADCTrajectory;

class QpSplineStSpeedOptimizerTest : public ::testing::Test {
  virtual void SetUp() {
    FLAGS_planning_config_file =
        "modules/planning/testdata/qp_spline_st_speed/planning_config.pb.txt";

    LoadPlanningConfig();
    optimizer_.reset(new QpSplineStSpeedOptimizer("ST_SPEED"));
    planning_pb = LoadPlanningPb(
        "modules/planning/testdata"
            "/qp_spline_st_speed/25_apollo_planning.pb.txt");
    InitFrame();
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

  void InitFrame() {
    frame.reset(new Frame(1));
    hdmap::PncMap* pnc_map;
    pnc_map = new hdmap::PncMap("modules/planning/testdata/base_map.txt");
    Frame::SetMap(pnc_map);

    frame->SetVehicleInitPose(
        planning_pb.debug().planning_data().adc_position().pose());
    frame->SetRoutingResponse(
        planning_pb.debug().planning_data().routing());

    frame->Init(planning_config_);
    frame->SetPlanningStartPoint(planning_pb.debug()
                                     .planning_data().init_point());

    PathData* pathData = frame->mutable_planning_data()->mutable_path_data();
    pathData->SetReferenceLine(&frame->reference_line());
    pathData->SetDiscretizedPath(getDiscretizedPath());
  }

  DiscretizedPath getDiscretizedPath() {
    DiscretizedPath path;

    common::Path qp_path_ground_truth;
    for (common::Path path : planning_pb.debug().planning_data().path()) {
      if (path.name() == "QP_SPLINE_PATH_OPTIMIZER") {
        qp_path_ground_truth = path;
      }
    }
    std::vector<common::PathPoint> pathPoints;
    for (common::PathPoint point : qp_path_ground_truth.path_point()) {
      pathPoints.push_back(point);
    }
    path.set_path_points(pathPoints);
    return path;
  }

  double timestamp_ = 0.0;
  std::unique_ptr<Frame> frame;
  PlanningPb planning_pb;
  std::unique_ptr<Optimizer> optimizer_;
  PlanningConfig planning_config_;
};

TEST_F(QpSplineStSpeedOptimizerTest, Process) {
  PlanningData* planning_data = frame->mutable_planning_data();
  EXPECT_EQ(planning_data->speed_data().speed_vector().size()
  , 0);
  optimizer_->Init(planning_config_);
  optimizer_->Optimize(frame.get());

  planning_internal::STGraphDebug st_graph_ground_truth;
  for (planning_internal::STGraphDebug stGraphDebug :
      planning_pb.debug().planning_data().st_graph()) {
    if (stGraphDebug.name() == "QP_SPLINE_ST_SPEED_OPTIMIZER") {
      st_graph_ground_truth = stGraphDebug;
    }
  }

  EXPECT_EQ(st_graph_ground_truth.speed_profile().size(), 162);
  EXPECT_EQ(frame->planning_data().speed_data().speed_vector()
                .size(), 162);

  for (std::int32_t i = 0;
       i < st_graph_ground_truth.speed_profile().size(); i++) {
    common::SpeedPoint ground_truth_point =
        st_graph_ground_truth.speed_profile().Get(i);
    common::SpeedPoint computed_point = planning_data
        ->speed_data().speed_vector()[i];
    EXPECT_NEAR(ground_truth_point.s(), computed_point.s(), 1.0e-3);
    EXPECT_NEAR(ground_truth_point.v(), computed_point.v(), 1.0e-3);
  }
}

}  // namespace planning
}  // namespace apollo
