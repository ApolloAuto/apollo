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

#include "modules/planning/planning_base/common/path/discretized_path.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::util::PointFactory;

TEST(DiscretizedPathTest, basic_test) {
  const double s1 = 0.0;
  const double s2 = s1 + std::sqrt(1.0 + 1.0);
  const double s3 = s2 + std::sqrt(1.0 + 1.0);
  const double s4 = s3 + std::sqrt(1.0 + 1.0);
  PathPoint p1 = PointFactory::ToPathPoint(0.0, 0.0, 0.0, s1);
  PathPoint p2 = PointFactory::ToPathPoint(1.0, 1.0, 0.0, s2);
  PathPoint p3 = PointFactory::ToPathPoint(2.0, 2.0, 0.0, s3);
  PathPoint p4 = PointFactory::ToPathPoint(3.0, 3.0, 0.0, s4);

  std::vector<PathPoint> path_points{p1, p2, p3, p4};

  DiscretizedPath discretized_path(path_points);
  EXPECT_EQ(discretized_path.size(), 4);

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

  discretized_path.clear();
  EXPECT_EQ(discretized_path.size(), 0);
}

TEST(DiscretizedPathTest, reverse_case) {
  const double s1 = 0.0;
  const double s2 = s1 - std::sqrt(1.0 + 1.0);
  const double s3 = s2 - std::sqrt(1.0 + 1.0);
  const double s4 = s3 - std::sqrt(1.0 + 1.0);
  PathPoint p1 = PointFactory::ToPathPoint(0.0, 0.0, 0.0, s1);
  PathPoint p2 = PointFactory::ToPathPoint(1.0, 1.0, 0.0, s2);
  PathPoint p3 = PointFactory::ToPathPoint(2.0, 2.0, 0.0, s3);
  PathPoint p4 = PointFactory::ToPathPoint(3.0, 3.0, 0.0, s4);

  std::vector<PathPoint> path_points{p1, p2, p3, p4};

  DiscretizedPath discretized_path(path_points);
  EXPECT_EQ(discretized_path.size(), 4);

  EXPECT_DOUBLE_EQ(discretized_path.Length(), -std::sqrt(1.0 + 1.0) * 3.0);

  auto eval_p1 = discretized_path.EvaluateReverse(0.0);
  EXPECT_DOUBLE_EQ(eval_p1.s(), 0.0);
  EXPECT_DOUBLE_EQ(eval_p1.x(), 0.0);
  EXPECT_DOUBLE_EQ(eval_p1.y(), 0.0);

  auto eval_p2 = discretized_path.EvaluateReverse(-0.3 * std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(eval_p2.s(), -0.3 * std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(eval_p2.x(), 0.3);
  EXPECT_DOUBLE_EQ(eval_p2.y(), 0.3);

  auto eval_p3 = discretized_path.EvaluateReverse(-1.8);
  EXPECT_DOUBLE_EQ(eval_p3.s(), -1.8);
  EXPECT_DOUBLE_EQ(eval_p3.x(), (1.0 + 0.8) / std::sqrt(2));
  EXPECT_DOUBLE_EQ(eval_p3.y(), (1.0 + 0.8) / std::sqrt(2));

  auto eval_p4 = discretized_path.EvaluateReverse(-2.5);
  EXPECT_DOUBLE_EQ(eval_p4.s(), -2.5);
  EXPECT_DOUBLE_EQ(eval_p4.x(), (2.0 + 0.5) / std::sqrt(2));
  EXPECT_DOUBLE_EQ(eval_p4.y(), (2.0 + 0.5) / std::sqrt(2));

  discretized_path.clear();
  EXPECT_EQ(discretized_path.size(), 0);
}

}  // namespace planning
}  // namespace apollo
