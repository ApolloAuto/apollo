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

#include "modules/localization/msf/common/util/frame_transform.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

TEST(FrameTransformTestSuite, LatlonToUtmXYTest) {
  double lon_rad = -2.129343746458001;
  double lat_rad = 0.6530018835651807;
  UTMCoor utm_xy;
  EXPECT_TRUE(FrameTransform::LatlonToUtmXY(lon_rad, lat_rad, &utm_xy));
  EXPECT_LT(std::fabs(utm_xy.x - 588278.9834174265), 1e-5);
  EXPECT_LT(std::fabs(utm_xy.y - 4141295.255870659), 1e-5);
}

TEST(FrameTransformTestSuite, UtmXYToLatlonTest) {
  double x = 588278.9834174265;
  double y = 4141295.255870659;
  int zone = 10;
  bool southhemi = false;
  WGS84Corr latlon;
  EXPECT_TRUE(FrameTransform::UtmXYToLatlon(x, y, zone, southhemi, &latlon));
  EXPECT_LT(std::fabs(latlon.log + 2.129343746458001), 1e-5);
  EXPECT_LT(std::fabs(latlon.lat - 0.6530018835651807), 1e-5);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
