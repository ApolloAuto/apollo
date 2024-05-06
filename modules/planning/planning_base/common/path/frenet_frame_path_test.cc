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

#include "modules/planning/planning_base/common/path/frenet_frame_path.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using apollo::common::FrenetFramePoint;

class FrenetFramePathTest : public ::testing::Test {
 public:
  void InitFrenetFramePath() {
    std::vector<double> s{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<double> l{1, 2, 1, 0, -1, -2, -1, 0, 1, 2};
    std::vector<double> dl{1, 0, -1, 0, -1, 0, 1, 1, 1, 0};
    std::vector<double> ddl{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<FrenetFramePoint> sl_points;
    for (size_t i = 0; i < s.size(); ++i) {
      sl_points.emplace_back();
      FrenetFramePoint& point = sl_points.back();
      point.set_s(s[i]);
      point.set_l(l[i]);
      point.set_dl(dl[i]);
      point.set_ddl(ddl[i]);
    }
    path_.reset(new FrenetFramePath(std::move(sl_points)));
  }
  void SetUp() { InitFrenetFramePath(); }

  std::unique_ptr<FrenetFramePath> path_;
};

TEST_F(FrenetFramePathTest, GetNearestPoint) {
  SLBoundary sl_boundary;
  {
    // at the beginning
    sl_boundary.set_start_s(-2);
    sl_boundary.set_end_s(-1);
    sl_boundary.set_start_l(0);
    sl_boundary.set_end_l(1);
    auto nearest = path_->GetNearestPoint(sl_boundary);
    EXPECT_DOUBLE_EQ(nearest.s(), 1.0);
    EXPECT_DOUBLE_EQ(nearest.l(), 1.0);
    EXPECT_DOUBLE_EQ(nearest.dl(), 1.0);
    EXPECT_DOUBLE_EQ(nearest.ddl(), 0.0);
  }
  {
    // at the end
    sl_boundary.set_start_s(11);
    sl_boundary.set_end_s(12);
    sl_boundary.set_start_l(0);
    sl_boundary.set_end_l(1);
    auto nearest = path_->GetNearestPoint(sl_boundary);
    EXPECT_DOUBLE_EQ(nearest.s(), 10.0);
    EXPECT_DOUBLE_EQ(nearest.l(), 2.0);
    EXPECT_DOUBLE_EQ(nearest.dl(), 0.0);
    EXPECT_DOUBLE_EQ(nearest.ddl(), 0.0);
  }

  {
    // in the middle, left side
    sl_boundary.set_start_s(1);
    sl_boundary.set_end_s(9);
    sl_boundary.set_start_l(3);
    sl_boundary.set_end_l(4);
    auto nearest = path_->GetNearestPoint(sl_boundary);
    EXPECT_DOUBLE_EQ(nearest.s(), 2.0);
    EXPECT_DOUBLE_EQ(nearest.l(), 2.0);
    EXPECT_DOUBLE_EQ(nearest.dl(), 0.0);
    EXPECT_DOUBLE_EQ(nearest.ddl(), 0.0);
  }
  {
    // in the middle, right side
    sl_boundary.set_start_s(1);
    sl_boundary.set_end_s(9);
    sl_boundary.set_start_l(-4);
    sl_boundary.set_end_l(-3);
    auto nearest = path_->GetNearestPoint(sl_boundary);
    EXPECT_DOUBLE_EQ(nearest.s(), 6.0);
    EXPECT_DOUBLE_EQ(nearest.l(), -2.0);
    EXPECT_DOUBLE_EQ(nearest.dl(), 0.0);
    EXPECT_DOUBLE_EQ(nearest.ddl(), 0.0);
  }
  {
    // in the middle, cross
    sl_boundary.set_start_s(1);
    sl_boundary.set_end_s(9);
    sl_boundary.set_start_l(-1);
    sl_boundary.set_end_l(0);
    auto nearest = path_->GetNearestPoint(sl_boundary);
    EXPECT_DOUBLE_EQ(nearest.s(), 4.0);
    EXPECT_DOUBLE_EQ(nearest.l(), 0.0);
    EXPECT_DOUBLE_EQ(nearest.dl(), 0.0);
    EXPECT_DOUBLE_EQ(nearest.ddl(), 0.0);
  }
}

}  // namespace planning
}  // namespace apollo
