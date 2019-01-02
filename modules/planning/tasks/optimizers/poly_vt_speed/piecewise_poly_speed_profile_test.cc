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

#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_speed_profile.h"

#include "gtest/gtest.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_speed_curve.h"

using apollo::common::SpeedPoint;

namespace apollo {
namespace planning {

TEST(PiecewisePolySpeedProfileTEST, speed_profile_test) {
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

  PiecewisePolySpeedProfile curve_profile;

  curve_profile.set_curve(curve);
  curve_profile.GeneratePoints(100);

  const auto& speed_points = curve_profile.eval_points();
  EXPECT_EQ(speed_points.size(), 100);
}

}  // namespace planning
}  // namespace apollo
