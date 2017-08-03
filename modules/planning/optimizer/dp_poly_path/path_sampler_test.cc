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

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/integration_tests/planning_test_base.h"
#include "modules/planning/optimizer/dp_poly_path/path_sampler.h"

namespace apollo {
namespace planning {

using common::adapter::AdapterManager;

class PathSamplerTest : public PlanningTestBase {
 public:
  virtual void SetUp() {
    PlanningTestBase::SetUp();
    sampled_points_.clear();
    init_point_.mutable_path_point()->set_x(pose_.position().x());
    init_point_.mutable_path_point()->set_y(pose_.position().y());
    init_point_.mutable_path_point()->set_z(0.0);
    init_point_.set_a(0.0);
    init_point_.set_v(0.0);
    const auto* frame = planning_.GetFrame();
    ASSERT_TRUE(frame != nullptr);
    reference_line_ = &frame->reference_line();
  }

 protected:
  std::vector<std::vector<common::SLPoint>> sampled_points_;
  common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
};

TEST_F(PathSamplerTest, sample_one_point) {
  dp_poly_path_config_.set_sample_points_num_each_level(1);
  PathSampler sampler(dp_poly_path_config_);
  bool sample_result =
      sampler.sample(*reference_line_, init_point_, &sampled_points_);
  EXPECT_TRUE(sample_result);
  // export_sl_points(sampled_points_, "/tmp/points.txt");
  ASSERT_EQ(8, sampled_points_.size());
  ASSERT_EQ(1, sampled_points_[0].size());
  ASSERT_EQ(1, sampled_points_[7].size());
  EXPECT_FLOAT_EQ(72.00795, sampled_points_[7][0].s());
  EXPECT_FLOAT_EQ(0, sampled_points_[7][0].l());
}

TEST_F(PathSamplerTest, sample_three_points) {
  ReferenceLine ref = *reference_line_;
  dp_poly_path_config_.set_sample_points_num_each_level(3);
  PathSampler sampler(dp_poly_path_config_);
  bool sample_result =
      sampler.sample(*reference_line_, init_point_, &sampled_points_);
  EXPECT_TRUE(sample_result);
  export_sl_points(sampled_points_, "/tmp/points.txt");
  ASSERT_EQ(8, sampled_points_.size());
  ASSERT_EQ(1, sampled_points_[0].size());
  ASSERT_EQ(3, sampled_points_[7].size());
  EXPECT_FLOAT_EQ(72.00795, sampled_points_[7][0].s());
  EXPECT_FLOAT_EQ(-0.5, sampled_points_[7][0].l());
  EXPECT_FLOAT_EQ(72.00795, sampled_points_[7][1].s());
  EXPECT_FLOAT_EQ(0, sampled_points_[7][1].l());
  EXPECT_FLOAT_EQ(72.00795, sampled_points_[7][2].s());
  EXPECT_FLOAT_EQ(0.5, sampled_points_[7][2].l());
}

}  // namespace planning
}  // namespace apollo
