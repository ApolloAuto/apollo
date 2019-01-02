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

/**
 * file : piecewise_poly_vt_speed_sampler_test.cc
 */
#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_vt_speed_sampler.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/poly_vt_speed_config.pb.h"

namespace apollo {
namespace planning {

class PiecewisePolyVTSpeedSamplerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_total_time(8.0);
    config_.set_total_s(150);
    config_.set_num_t_layers(8);
    config_.set_online_num_v_layers(21);
    config_.set_matrix_dim_s(400);
    config_.set_online_max_acc(2);
    config_.set_online_max_dec(-4);
    config_.set_online_max_speed(14);
    config_.set_offline_num_v_layers(60);
    config_.set_offline_max_acc(5);
    config_.set_offline_max_dec(-6.0);
    config_.set_offline_max_speed(40);
    config_.set_num_evaluated_points(17);
    config_.set_sampling_unit_v(0.5);
    config_.set_max_sampling_unit_v(0.6);

    common::TrajectoryPoint init_point_;
    init_point_.set_v(0.0);
    init_point_.set_a(0.0);
  }

 protected:
  PolyVTSpeedConfig config_;
  common::TrajectoryPoint init_point_;
};

TEST_F(PiecewisePolyVTSpeedSamplerTest, sampling) {
  PiecewisePolyVTSpeedSampler sampler(config_);
  std::vector<PiecewisePolySpeedProfile> samples;
  bool suc = sampler.Sample(init_point_, 140, &samples);
  EXPECT_TRUE(suc);
  EXPECT_EQ(samples.size(), 845);
  auto one_sample = samples[234];
  EXPECT_EQ(one_sample.eval_points().size(), 17);
  one_sample.GeneratePoints(100);
  const auto& points = one_sample.eval_points();
  EXPECT_EQ(points.size(), 100);
}

}  // namespace planning
}  // namespace apollo
