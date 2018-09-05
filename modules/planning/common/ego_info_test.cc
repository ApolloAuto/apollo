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

#include "modules/planning/common/ego_info.h"

#include "gtest/gtest.h"

#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class EgoInfoTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(EgoInfoTest, simple) {
  const auto p =
      common::util::MakePathPoint(1.23, 3.23, 52.18, 0.1, 0.3, 0.32, 0.4);
  common::TrajectoryPoint tp;
  tp.mutable_path_point()->CopyFrom(p);
  auto* ego_info = EgoInfo::instance();
  ego_info->set_start_point(tp);
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().x(), p.x());
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().y(), p.y());
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().z(), p.z());

  ReferenceLine ref_line;
  SLBoundary sl_boundary;
  sl_boundary.set_start_s(10.22);
  sl_boundary.set_end_s(28.79);
  ego_info->SetSLBoundary(&ref_line, sl_boundary);

  SLBoundary sl_boundary2 = ego_info->GetSLBoundaryOnReferenceLine(&ref_line);

  EXPECT_DOUBLE_EQ(sl_boundary2.start_s(), sl_boundary.start_s());
  EXPECT_DOUBLE_EQ(sl_boundary2.end_s(), sl_boundary.end_s());
}

}  // namespace planning
}  // namespace apollo
