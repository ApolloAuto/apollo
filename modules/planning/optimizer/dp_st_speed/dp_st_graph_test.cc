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
#include <iostream>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/proto/dp_st_speed_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/optimizer/dp_st_speed/dp_st_graph.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

class DpStSpeedTest : public PlanningTestBase {
 public:
  void SetPathDataWithStraightLine() {
    double l = 0.0;
    const double ds = 1.0;

    std::vector<common::FrenetFramePoint> points;
    for (double s = 0.0; s < 100.0; s += ds) {
      common::FrenetFramePoint ffp;
      ffp.set_s(s);
      ffp.set_l(l);
      points.push_back(ffp);
    }

    FrenetFramePath path(points);
    path_data_.set_frenet_path(path);
    path_data_.set_reference_line(reference_line_);
  }

  virtual void SetUp() {
    google::InitGoogleLogging("DpStSpeedTest");
    PlanningTestBase::SetUp();
    const auto* frame = planning_.GetFrame();
    reference_line_ = &(frame->reference_line());
    SetPathDataWithStraightLine();
  }

 protected:
  const ReferenceLine* reference_line_ = nullptr;
  common::TrajectoryPoint init_point_;
  SpeedData speed_data_;  // output
  PathData path_data_;    // input
};

TEST_F(DpStSpeedTest, dp_st_graph_test) {
  DpStSpeedConfig dp_st_speed_config;
  DpStGraph dp_st_graph(dp_st_speed_config, path_data_);
}

}  // namespace planning
}  // namespace apollo
