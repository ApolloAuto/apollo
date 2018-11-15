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
#include "modules/dreamview/backend/hmi/vehicle_manager.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"

DECLARE_string(vehicle_data_config_filename);

namespace apollo {
namespace dreamview {
using apollo::common::util::StrCat;

static const char kTestVehicle[] =
    "modules/dreamview/backend/hmi/testdata/vehicle";
static const char kTargetDir[] = "/tmp/vehicle";

class VehicleManagerTest : public ::testing::Test {
 protected:
  VehicleManagerTest() {
    // According to this config file, vehicle_data.pb.txt will be copied to
    // kTargetDir.
    FLAGS_vehicle_data_config_filename =
        StrCat(kTestVehicle, "/vehicle_data.pb.txt");
  }
};

TEST_F(VehicleManagerTest, Failure) {
  EXPECT_FALSE(VehicleManager::instance()->UseVehicle("/somewhere/bad"));
}

TEST_F(VehicleManagerTest, Success) {
  ASSERT_TRUE(apollo::common::util::EnsureDirectory(kTargetDir));

  EXPECT_TRUE(VehicleManager::instance()->UseVehicle(kTestVehicle));
  EXPECT_TRUE(apollo::common::util::PathExists(
      StrCat(kTargetDir, "/vehicle_data.pb.txt")));

  ASSERT_TRUE(apollo::common::util::RemoveAllFiles(kTargetDir));
}

}  // namespace dreamview
}  // namespace apollo
