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

#include <unistd.h>
#include <climits>
#include <fstream>
#include <sstream>

#include "google/protobuf/text_format.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/third_party_perception/integration_tests/third_party_perception_test_base.h"
#include "ros/include/ros/ros.h"

DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_string(test_localization_file, "", "localization input file");
DEFINE_string(test_monitor_file, "", "montor input file");

DEFINE_bool(test_update_golden_log, false, "true to update golden log file.");

namespace apollo {
namespace third_party_perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessage;

uint32_t ThirdPartyPerceptionTestBase::s_seq_num_ = 0;

void ThirdPartyPerceptionTestBase::SetUpTestCase() {
  ros::Time::init();
  FLAGS_adapter_config_filename =
      "modules/third_party_perception/testdata/conf/adapter.conf";
}

void ThirdPartyPerceptionTestBase::SetUp() { ++s_seq_num_; }

}  // namespace third_party_perception
}  // namespace apollo
