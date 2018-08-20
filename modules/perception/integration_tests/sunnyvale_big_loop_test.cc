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

#include <string>

#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/integration_tests/perception_test_base.h"

namespace apollo {
namespace perception {

using apollo::common::PointENU;
using apollo::common::time::Clock;

/**
 * @class SunnyvaleBigLoopTest
 * @brief This is an integration test that uses the sunnyvale_big_loop map.
 */

class SunnyvaleBigLoopTest : public PerceptionTestBase {
 public:
  virtual void SetUp() {
    FLAGS_map_dir = "modules/map/data/sunnyvale_big_loop";
    FLAGS_test_base_map_filename = "base_map.bin";
    AERROR << "SetUp done.";
  }
};

TEST_F(SunnyvaleBigLoopTest, test_01) {
  std::string seq_num = "1";
  FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
  FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";
  PerceptionTestBase::SetUp();

  // TODO(All): implement sunnyvale big loop test here.
  AERROR << "test done.";
}

}  // namespace perception
}  // namespace apollo

TMAIN;
