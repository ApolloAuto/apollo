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

#include "modules/common/util/time_util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

TEST(TimeUtilTest, TestUnix2Gps) {
  double unix_time = 1476761767;
  double gps_time = TimeUtil::Unix2Gps(unix_time);
  EXPECT_NEAR(gps_time, 1160796984, 0.000001);

  double unix_time1 = 1483228799;
  double gps_time1 = TimeUtil::Unix2Gps(unix_time1);
  EXPECT_NEAR(gps_time1, 1167264017, 0.000001);
}

TEST(TimeUtilTest, TestGps2Unix) {
  double gps_time = 1160796984;
  double unix_time = TimeUtil::Gps2Unix(gps_time);
  EXPECT_NEAR(unix_time, 1476761767, 0.000001);
  double gps_time1 = 1260796984;
  double unix_time1 = TimeUtil::Gps2Unix(gps_time1);
  EXPECT_NEAR(unix_time1, 1576761766, 0.000001);
}

}  // namespace util
}  // namespace common
}  // namespace apollo
