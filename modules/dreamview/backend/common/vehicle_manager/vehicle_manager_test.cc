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
#include "modules/dreamview/backend/common/vehicle_manager/vehicle_manager.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "gtest/gtest.h"

namespace apollo {
namespace dreamview {

static const char kTestVehicle[] =
    "modules/dreamview/backend/common/vehicle_manager/testdata/vehicle";
static const char kTargetDir[] = "/tmp/vehicle";

class VehicleManagerTest : public ::testing::Test {
 protected:
  VehicleManagerTest() {
    // According to this config file, vehicle_data.pb.txt will be copied to
    // kTargetDir.
    FLAGS_vehicle_data_config_filename =
        absl::StrCat(kTestVehicle, "/vehicle_data.pb.txt");
  }
};

TEST_F(VehicleManagerTest, Failure) {
  EXPECT_FALSE(VehicleManager::Instance()->UseVehicle("/somewhere/bad"));
}

TEST_F(VehicleManagerTest, Success) {
  ASSERT_TRUE(cyber::common::EnsureDirectory(kTargetDir));

  EXPECT_TRUE(VehicleManager::Instance()->UseVehicle(kTestVehicle));
  EXPECT_TRUE(cyber::common::PathExists(
      absl::StrCat(kTargetDir, "/vehicle_data.pb.txt")));

  ASSERT_TRUE(cyber::common::RemoveAllFiles(kTargetDir));
}

}  // namespace dreamview
}  // namespace apollo
