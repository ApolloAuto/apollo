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

#include "modules/map/pnc_map/cuda_util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace pnc_map {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

TEST(CudaUtil, CudaNearestSegment) {
  CudaNearestSegment segment_tool;
  Vec2d p1(0, 0);
  Vec2d p2(1, 0);
  Vec2d p3(2, 0);
  Vec2d p4(3, 0);

  std::vector<LineSegment2d> segments;
  segments.emplace_back(p1, p2);
  segments.emplace_back(p2, p3);
  segments.emplace_back(p3, p4);

  segment_tool.UpdateLineSegment(segments);
  int nearest_index = segment_tool.FindNearestSegment(0.5, 1.0);
  EXPECT_EQ(0, nearest_index);
  nearest_index = segment_tool.FindNearestSegment(1.5, 1.0);
  EXPECT_EQ(1, nearest_index);
}

}  // namespace pnc_map
}  // namespace apollo
