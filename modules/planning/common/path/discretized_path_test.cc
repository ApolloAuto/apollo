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

#include "modules/planning/common/path/discretized_path.h"

#include <vector>

#include "gtest/gtest.h"

#include "modules/common/log.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::util::MakePathPoint;

TEST(DiscretizedPathTest, basic_test) {
  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(std::sqrt(1.0 + 1.0) + p1.s());
  PathPoint p3 = MakePathPoint(2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p3.set_s(std::sqrt(1.0 + 1.0) + p2.s());
  PathPoint p4 = MakePathPoint(3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p4.set_s(std::sqrt(1.0 + 1.0) + p3.s());

  std::vector<PathPoint> path_points{p1, p2, p3, p4};

  DiscretizedPath discretized_path(path_points);
  EXPECT_EQ(discretized_path.NumOfPoints(), 4);

  EXPECT_DOUBLE_EQ(discretized_path.Length(), std::sqrt(1.0 + 1.0) * 3.0);

  auto eval_p1 = discretized_path.Evaluate(0.0);
  EXPECT_DOUBLE_EQ(eval_p1.s(), 0.0);
  EXPECT_DOUBLE_EQ(eval_p1.x(), 0.0);
  EXPECT_DOUBLE_EQ(eval_p1.y(), 0.0);

  auto eval_p2 = discretized_path.Evaluate(0.3 * std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(eval_p2.s(), 0.3 * std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(eval_p2.x(), 0.3);
  EXPECT_DOUBLE_EQ(eval_p2.y(), 0.3);

  auto eval_p3 = discretized_path.Evaluate(1.8);
  EXPECT_DOUBLE_EQ(eval_p3.s(), 1.8);
  EXPECT_DOUBLE_EQ(eval_p3.x(), (1.0 + 0.8) / std::sqrt(2));
  EXPECT_DOUBLE_EQ(eval_p3.y(), (1.0 + 0.8) / std::sqrt(2));

  auto eval_p4 = discretized_path.Evaluate(2.5);
  EXPECT_DOUBLE_EQ(eval_p4.s(), 2.5);
  EXPECT_DOUBLE_EQ(eval_p4.x(), (2.0 + 0.5) / std::sqrt(2));
  EXPECT_DOUBLE_EQ(eval_p4.y(), (2.0 + 0.5) / std::sqrt(2));

  auto points = discretized_path.path_points();
  EXPECT_EQ(points.size(), path_points.size());

  EXPECT_DOUBLE_EQ(discretized_path.StartPoint().s(), path_points.front().s());
  EXPECT_DOUBLE_EQ(discretized_path.EndPoint().s(), path_points.back().s());

  discretized_path.Clear();
  EXPECT_EQ(discretized_path.NumOfPoints(), 0);
}

}  // namespace planning
}  // namespace apollo
