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

#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_config.h"

#include <string>
#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

TEST(MapNdtConfigTestSuite, config) {
  // init config
  NdtMapConfig config("map_ndt_v01");
  EXPECT_EQ(config.map_version_, "map_ndt_v01");
  EXPECT_TRUE(config.map_is_compression_);

  // resolution
  config.SetSingleResolutionZ(1.0);
  EXPECT_DOUBLE_EQ(config.map_resolutions_z_[0], 1.0);
  config.SetMultiResolutionsZ();
  EXPECT_EQ(config.map_resolutions_z_.size(), 10);
  config.map_datasets_.push_back("map_ndt");

  // Save & Load
  const std::string config_file = "/tmp/config_ndt_test.xml";
  EXPECT_TRUE(config.Save(config_file));
  EXPECT_TRUE(config.Load(config_file));
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
