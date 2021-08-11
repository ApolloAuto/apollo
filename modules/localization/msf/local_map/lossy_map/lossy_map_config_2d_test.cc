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

#include "modules/localization/msf/local_map/lossy_map/lossy_map_config_2d.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief Test load and set methods in LossyMapConfig2D. */
TEST(LossyMapConfig2DTestSuite, LoadSetTest) {
  BaseMapConfig config("lossy_map");
  EXPECT_TRUE(
      config.Load("/apollo/modules/localization/msf/local_map/"
                  "test_data/lossy_single_map/config.xml"));
  config.SetMultiResolutions();
  EXPECT_EQ(config.map_resolutions_.size(), 10);
  EXPECT_DOUBLE_EQ(config.map_resolutions_[0], 0.03125);

  config.ResizeMapRange();
  EXPECT_DOUBLE_EQ(config.map_range_.GetMinX(), -16.0 * 1024);
  EXPECT_DOUBLE_EQ(config.map_range_.GetMinY(), -16.0 * 1024);
  EXPECT_DOUBLE_EQ(config.map_range_.GetMaxX(), 66.0 * 16.0 * 1024);
  EXPECT_DOUBLE_EQ(config.map_range_.GetMaxY(), 630.0 * 16.0 * 1024);

  config.SetSingleResolutions();
  EXPECT_EQ(config.map_resolutions_.size(), 1);
  EXPECT_DOUBLE_EQ(config.map_resolutions_[0], 0.125);

  BaseMapConfig config2;
  EXPECT_FALSE(
      config2.Load("/apollo/modules/localization/msf/local_map/"
                   "test_data/lossy_single_map/config.xml"));
}

/**@brief Test save method. */
TEST(LossyMapConfig2DTestSuite, SaveTest) {
  BaseMapConfig config("lossy_map");
  EXPECT_TRUE(
      config.Load("/apollo/modules/localization/msf/local_map/"
                  "test_data/lossy_single_map/config.xml"));
  EXPECT_TRUE(config.Save("/tmp/temp_output_file.xml"));
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
