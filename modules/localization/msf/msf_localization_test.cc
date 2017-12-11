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
#define private public
#include "modules/localization/msf/msf_localization.h"

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/localization/common/localization_gflags.h"

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;

namespace apollo {
namespace localization {

class MSFLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    msf_localizatoin_.reset(new MSFLocalization());

    // Setup AdapterManager.
    AdapterManagerConfig config;
    config.set_is_ros(false);
    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
      sub_config->set_type(AdapterConfig::LOCALIZATION);
    }
    AdapterManager::Init(config);
  }

 protected:
  std::unique_ptr<MSFLocalization> msf_localizatoin_;
};

// TEST_F(MSFLocalizationTest, InitParams) {
// FLAGS_imuant_from_gnss_conf_file = false;
// FLAGS_imu_to_ant_offset_x = 5;
// msf_localizatoin_->InitParams();
// double imu_ant_offset_x =
//     (msf_localizatoin_->localizaiton_param_).imu_to_ant_offset.offset_x;
// ASSERT_LT(std::abs(imu_ant_offset_x - 5.0), 1e-5);
// }

}  // namespace localization
}  // namespace apollo
