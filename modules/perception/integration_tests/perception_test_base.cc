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

#include "modules/perception/integration_tests/perception_test_base.h"

#include <cstdlib>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace perception {

using common::adapter::AdapterManager;

DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_string(test_pointcloud_file, "", "The pointcloud file used in test");
DEFINE_string(test_localization_file, "", "The localization test file");
DEFINE_string(test_chassis_file, "", "The chassis test file");
DEFINE_string(perception_config_file, "", "The perception config file");
DEFINE_string(perception_adapter_config_filename, "",
              "The perception adapter config file");

void PerceptionTestBase::SetUpTestCase() {
  FLAGS_perception_config_file =
      "modules/perception/conf/perception_config.pb.txt";
  FLAGS_perception_adapter_config_filename =
      "modules/perception/integration_test/testdata/conf/adapter.conf";

  FLAGS_test_pointcloud_file = "";
  FLAGS_test_localization_file = "";
  FLAGS_test_chassis_file = "";
}

#define FEED_ADAPTER(TYPE, FILENAME)                                           \
  if (!AdapterManager::Get##TYPE()) {                                          \
    AERROR << #TYPE                                                            \
        " is not registered in adapter manager, check adapter file "           \
           << FLAGS_perception_adapter_config_filename;                        \
    return false;                                                              \
  }                                                                            \
  if (!FILENAME.empty()) {                                                     \
    if (!AdapterManager::Feed##TYPE##File(FLAGS_test_data_dir + "/" +          \
                                          FILENAME)) {                         \
      AERROR << "Failed to feed " #TYPE " file " << FLAGS_test_data_dir << "/" \
             << FILENAME;                                                      \
      return false;                                                            \
    }                                                                          \
    AINFO << "Using " #TYPE << " provided by " << FLAGS_test_data_dir << "/"   \
          << FILENAME;                                                         \
  }

bool PerceptionTestBase::SetUpAdapters() {
  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_perception_adapter_config_filename);
  }
  FEED_ADAPTER(PointCloud, FLAGS_test_localization_file);
  FEED_ADAPTER(Localization, FLAGS_test_localization_file);
  FEED_ADAPTER(Chassis, FLAGS_test_chassis_file);
  return true;
}

void PerceptionTestBase::SetUp() {
  perception_.Stop();
  CHECK(SetUpAdapters()) << "Failed to setup adapters";
  CHECK(perception_.Init().ok()) << "Failed to init perception module";
}

void PerceptionTestBase::UpdateData() {
  CHECK(SetUpAdapters()) << "Failed to setup adapters";
}

bool PerceptionTestBase::RunPerception(const std::string& test_case_name,
                                       int case_num) {
  // TODO(All): Implement RunPerception here.
  return true;
}

}  // namespace perception
}  // namespace apollo
