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

/**
 * @file
 **/

#include "modules/planning/common/speed_limit.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using SpeedPoint = apollo::common::SpeedPoint;

class SpeedLimitTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    speed_limit_.Clear();
    for (int i = 0; i < 100; ++i) {
      SpeedPoint sp;
      sp.set_s(i * 1.0);
      sp.set_t(i * 0.5);
      sp.set_v((i % 2 == 0) ? 5.0 : 10.0);
      speed_limit_.AddSpeedLimit(std::move(sp));
    }
  }

 protected:
  SpeedLimit speed_limit_;
};

TEST_F(SpeedLimitTest, SimpleSpeedLimitCreation) {
  SpeedLimit simple_speed_limit;
  EXPECT_TRUE(simple_speed_limit.speed_points().empty());
  EXPECT_EQ(speed_limit_.speed_points().size(), 100);
}

TEST_F(SpeedLimitTest, GetSpeedLimitByS) {
  EXPECT_EQ(speed_limit_.speed_points().size(), 100);
  double s = 0.0;
  const double ds = 0.01;
  while (s < 99.0) {
    double v_limit = speed_limit_.GetSpeedLimitByS(s);

    auto it_lower = std::lower_bound(
        speed_limit_.speed_points().begin(), speed_limit_.speed_points().end(),
        s, [](const SpeedPoint& point, const double curr_s) {
          return point.s() < curr_s;
        });

    EXPECT_DOUBLE_EQ(v_limit, it_lower->v());
    s += ds;
  }
}

TEST_F(SpeedLimitTest, GetSpeedLimitByT) {
  EXPECT_EQ(speed_limit_.speed_points().size(), 100);
  double t = 0.0;
  const double dt = 0.01;
  while (t < 49.0) {
    double v_limit = speed_limit_.GetSpeedLimitByT(t);

    auto it_lower = std::lower_bound(
        speed_limit_.speed_points().begin(), speed_limit_.speed_points().end(),
        t, [](const SpeedPoint& point, const double curr_t) {
          return point.t() < curr_t;
        });

    EXPECT_DOUBLE_EQ(v_limit, it_lower->v());
    t += dt;
  }
}

}  // namespace planning
}  // namespace apollo
