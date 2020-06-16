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
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

TEST(EgoInfoTest, EgoInfoSimpleTest) {
  const auto p = common::util::PointFactory::ToPathPoint(1.23, 3.23, 52.18, 0.0,
                                                         0.1, 0.3, 0.32, 0.4);
  common::TrajectoryPoint tp;
  tp.mutable_path_point()->CopyFrom(p);
  auto ego_info = std::make_unique<EgoInfo>();
  ego_info->set_start_point(tp);
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().x(), p.x());
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().y(), p.y());
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().z(), p.z());

  uint32_t sequence_num = 0;
  common::TrajectoryPoint planning_start_point;
  common::VehicleState vehicle_state;
  ReferenceLineProvider reference_line_provider;

  LocalView dummy_local_view;
  Frame frame(sequence_num, dummy_local_view, planning_start_point,
              vehicle_state, &reference_line_provider);
  ego_info->CalculateFrontObstacleClearDistance(frame.obstacles());
}

}  // namespace planning
}  // namespace apollo
