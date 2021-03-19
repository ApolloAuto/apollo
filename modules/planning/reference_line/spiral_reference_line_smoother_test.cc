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

#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/reference_line_smoother_config.pb.h"

namespace apollo {
namespace planning {

class SpiralReferenceLineSmootherTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.mutable_spiral()->set_max_deviation(0.1);
    config_.mutable_spiral()->set_max_iteration(300);
    config_.mutable_spiral()->set_opt_tol(1.0e-6);
    config_.mutable_spiral()->set_opt_acceptable_tol(1.0e-4);

    config_.mutable_spiral()->set_weight_curve_length(1.0);
    config_.mutable_spiral()->set_weight_kappa(1.0);
    config_.mutable_spiral()->set_weight_dkappa(100.0);

    raw_points_.resize(21);
    raw_points_[0] = {4.946317773, 0.08436953512};
    raw_points_[1] = {5.017218975, 0.7205757236};
    raw_points_[2] = {4.734635316, 1.642930209};
    raw_points_[3] = {4.425064575, 2.365356462};
    raw_points_[4] = {3.960102096, 2.991632152};
    raw_points_[5] = {3.503172702, 3.44091492};
    raw_points_[6] = {2.989950824, 3.9590821};
    raw_points_[7] = {2.258523535, 4.554377368};
    raw_points_[8] = {1.562447892, 4.656801472};
    raw_points_[9] = {0.8764776599, 4.971705856};
    raw_points_[10] = {0.09899323097, 4.985845841};
    raw_points_[11] = {-0.7132021974, 5.010851105};
    raw_points_[12] = {-1.479055426, 4.680181989};
    raw_points_[13] = {-2.170306775, 4.463442715};
    raw_points_[14] = {-3.034455492, 4.074651273};
    raw_points_[15] = {-3.621987909, 3.585790302};
    raw_points_[16] = {-3.979289889, 3.014232351};
    raw_points_[17] = {-4.434628966, 2.367848826};
    raw_points_[18] = {-4.818245921, 1.467395733};
    raw_points_[19] = {-4.860190444, 0.8444358019};
    raw_points_[20] = {-5.09947597, -0.01022405467};
  }

  ReferenceLineSmootherConfig config_;
  std::vector<Eigen::Vector2d> raw_points_;
};

TEST_F(SpiralReferenceLineSmootherTest, smooth_stand_alone) {
  std::vector<double> theta;
  std::vector<double> kappa;
  std::vector<double> dkappa;
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;

  SpiralReferenceLineSmoother spiral_smoother(config_);
  int res = spiral_smoother.SmoothStandAlone(raw_points_, &theta, &kappa,
                                             &dkappa, &s, &x, &y);
  // TODO(Yajia): fix this test.
  EXPECT_LT(res, 100000);
}

}  // namespace planning
}  // namespace apollo
