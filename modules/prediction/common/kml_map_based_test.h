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

#ifndef MODULES_PREDICTION_COMMON_KML_MAP_BASED_TEST_H_
#define MODULES_PREDICTION_COMMON_KML_MAP_BASED_TEST_H_

#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"

namespace apollo {
namespace prediction {

class KMLMapBasedTest : public ::testing::Test {
 public:
  KMLMapBasedTest() {
    FLAGS_map_dir = "modules/prediction/testdata";
    FLAGS_base_map_filename = "kml_map.bin";
  }
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_KML_MAP_BASED_TEST_H_
