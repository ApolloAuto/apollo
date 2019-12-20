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

#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_config.h"

#include "gtest/gtest.h"

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

TEST(PyramidMapConfigTestSuite, base_config_method) {
  PyramidMapConfig config("lossy_full_alt");
  // case 1: no config file
  // std::string config_file = "pyramid_map_config_error.xml";
  // config.load(config_file);

  // case 2: flags are false
  config.has_intensity_ = false;
  config.has_intensity_var_ = false;
  config.has_altitude_ = false;
  config.has_altitude_var_ = false;
  config.has_ground_altitude_ = false;
  config.has_ground_count_ = false;
  config.has_count_ = false;

  std::string config_file = "pyramid_map_config.xml";
  config.Save(config_file);
  config.Load(config_file);

  // case 3: flags are true
  PyramidMapConfig config2("lossy_full_alt");
  config2.has_intensity_ = true;
  config2.has_intensity_var_ = true;
  config2.has_altitude_ = true;
  config2.has_altitude_var_ = true;
  config2.has_ground_altitude_ = true;
  config2.has_ground_count_ = true;
  config2.has_count_ = true;

  config_file = "pyramid_map_config2.xml";
  config2.Save(config_file);
  config2.Load(config_file);
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
