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

#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_speed_curve.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "gtest/gtest.h"

using apollo::common::SpeedPoint;

namespace apollo {
namespace planning {

TEST(PiecewisePolySpeedCurveTest, test_case_one) {
  SpeedPoint init_point;
  init_point.set_s(0.0);
  init_point.set_t(0.0);
  init_point.set_v(0.0);
  init_point.set_a(0.0);
  init_point.set_da(0.0);

  SpeedPoint connect_point;
  connect_point.set_s(0.0);
  connect_point.set_t(4.0);
  connect_point.set_v(8.0);
  connect_point.set_a(0.0);
  connect_point.set_da(0.0);

  double end_t = 8.0;
  PiecewisePolySpeedCurve curve(init_point, connect_point, end_t);
  std::vector<SpeedPoint> sampling_points;
  curve.SampleSpeedPoints(1, &sampling_points);

  EXPECT_EQ(1, sampling_points.size());
  EXPECT_NEAR(0.0, sampling_points[0].s(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[0].t(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[0].v(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[0].a(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[0].da(), 0.001);

  size_t num_points = 5;
  curve.SampleSpeedPoints(num_points, &sampling_points);
  EXPECT_EQ(sampling_points.size(), num_points);
  EXPECT_NEAR(0.0, sampling_points[0].s(), 0.001);
  EXPECT_NEAR(4.6, sampling_points[1].s(), 0.001);
  EXPECT_NEAR(19.2, sampling_points[2].s(), 0.001);
  EXPECT_NEAR(35.2, sampling_points[3].s(), 0.001);
  EXPECT_NEAR(51.2, sampling_points[4].s(), 0.001);

  EXPECT_NEAR(0.0, sampling_points[0].t(), 0.001);
  EXPECT_NEAR(2.0, sampling_points[1].t(), 0.001);
  EXPECT_NEAR(4.0, sampling_points[2].t(), 0.001);
  EXPECT_NEAR(6.0, sampling_points[3].t(), 0.001);
  EXPECT_NEAR(8.0, sampling_points[4].t(), 0.001);

  EXPECT_NEAR(0.0, sampling_points[0].v(), 0.001);
  EXPECT_NEAR(5.5, sampling_points[1].v(), 0.001);
  EXPECT_NEAR(8.0, sampling_points[2].v(), 0.001);
  EXPECT_NEAR(8.0, sampling_points[3].v(), 0.001);
  EXPECT_NEAR(8.0, sampling_points[4].v(), 0.001);

  EXPECT_NEAR(0.0, sampling_points[0].a(), 0.001);
  EXPECT_NEAR(3.0, sampling_points[1].a(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[2].a(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[3].a(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[4].a(), 0.001);

  EXPECT_NEAR(6.0, sampling_points[0].da(), 0.001);
  EXPECT_NEAR(-1.5, sampling_points[1].da(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[2].da(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[3].da(), 0.001);
  EXPECT_NEAR(0.0, sampling_points[4].da(), 0.001);
}
}  // namespace planning
}  // namespace apollo
