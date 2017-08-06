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

namespace apollo {
namespace planning {

class QpSplinePathOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_reference_line_smoother_config_file =
        "modules/planning/testdata/conf/reference_line_smoother_config.pb.txt";
    qp_spline_path_optimizer_.reset(new QpSplinePathOptimizer("QP_PATH"));
  }

 protected:
  std::unique_ptr<QpSplinePathOptimizer> qp_spline_path_optimizer_;
};

TEST_F(QpSplinePathOptimizerTest, Process) {
  Frame frame(1);

  EXPECT_EQ(qp_spline_path_optimizer_.get()->name(), "QP_PATH");
}

}  // namespace planning
}  // namespace apollo
