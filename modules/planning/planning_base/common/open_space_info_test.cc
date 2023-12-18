/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/planning/planning_base/common/open_space_info.h"

#include "gtest/gtest.h"

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/planning_msgs/sl_boundary.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace planning {

class OpenSpaceInfoTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  OpenSpaceInfo open_space_info_;
};

TEST_F(OpenSpaceInfoTest, Init) { EXPECT_NE(&open_space_info_, nullptr); }

bool ComputeSLBoundaryIntersection(const SLBoundary& sl_boundary,
                                   const double s, double* ptr_l_min,
                                   double* ptr_l_max);

}  // namespace planning
}  // namespace apollo
