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

#include "modules/localization/rtk/rtk_localization.h"

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

class RTKLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_v = 3;
    rtk_localizatoin_.reset(new RTKLocalization());

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
  template <class T>
  void load_data(const std::string &filename, T *data) {
    CHECK(common::util::GetProtoFromFile(filename, data))
        << "Failed to open file " << filename;
  }

  std::unique_ptr<RTKLocalization> rtk_localizatoin_;
};

TEST_F(RTKLocalizationTest, InterpolateIMU) {
  // timestamp inbetween + time_diff is big enough(>0.001), interpolate
  {
    apollo::localization::Imu imu1;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &imu1);

    apollo::localization::Imu imu2;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &imu2);

    apollo::localization::Imu expected_result;
    load_data("modules/localization/testdata/1_imu_result.pb.txt",
              &expected_result);

    apollo::localization::Imu imu;
    double timestamp = 1173545122.69;
    rtk_localizatoin_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }

  // timestamp inbetween + time_diff is too small(<0.001), no interpolate
  {
    apollo::localization::Imu imu1;
    load_data("modules/localization/testdata/2_imu_1.pb.txt", &imu1);

    apollo::localization::Imu imu2;
    load_data("modules/localization/testdata/2_imu_2.pb.txt", &imu2);

    apollo::localization::Imu expected_result;
    load_data("modules/localization/testdata/2_imu_result.pb.txt",
              &expected_result);

    apollo::localization::Imu imu;
    double timestamp = 1173545122.2001;
    rtk_localizatoin_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }

  // timestamp < imu1.timestamp
  {
    apollo::localization::Imu imu1;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &imu1);

    apollo::localization::Imu imu2;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &imu2);

    apollo::localization::Imu expected_result;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &expected_result);

    apollo::localization::Imu imu;
    double timestamp = 1173545122;
    rtk_localizatoin_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }

  // timestamp > imu2.timestamp
  {
    apollo::localization::Imu imu1;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &imu1);

    apollo::localization::Imu imu2;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &imu2);

    apollo::localization::Imu expected_result;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &expected_result);

    apollo::localization::Imu imu;
    double timestamp = 1173545122.70;
    rtk_localizatoin_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }
}

TEST_F(RTKLocalizationTest, ComposeLocalizationMsg) {
  // FLAGS_enable_map_reference_unify: false
  {
    FLAGS_enable_map_reference_unify = false;

    apollo::localization::Gps gps;
    load_data("modules/localization/testdata/3_gps_1.pb.txt", &gps);

    apollo::localization::Imu imu;
    load_data("modules/localization/testdata/3_imu_1.pb.txt", &imu);

    apollo::localization::LocalizationEstimate expected_result;
    load_data("modules/localization/testdata/3_localization_result_1.pb.txt",
              &expected_result);

    apollo::localization::LocalizationEstimate localization;
    rtk_localizatoin_->ComposeLocalizationMsg(gps, imu, &localization);

    EXPECT_EQ(1, localization.header().sequence_num());
    EXPECT_STREQ("localization", localization.header().module_name().c_str());
    EXPECT_STREQ(expected_result.pose().DebugString().c_str(),
                 localization.pose().DebugString().c_str());
  }

  // FLAGS_enable_map_reference_unify: true
  {
    FLAGS_enable_map_reference_unify = true;

    apollo::localization::Gps gps;
    load_data("modules/localization/testdata/3_gps_1.pb.txt", &gps);

    apollo::localization::Imu imu;
    load_data("modules/localization/testdata/3_imu_1.pb.txt", &imu);

    apollo::localization::LocalizationEstimate expected_result;
    load_data("modules/localization/testdata/3_localization_result_2.pb.txt",
              &expected_result);

    apollo::localization::LocalizationEstimate localization;
    rtk_localizatoin_->ComposeLocalizationMsg(gps, imu, &localization);

    EXPECT_EQ(2, localization.header().sequence_num());
    EXPECT_STREQ("localization", localization.header().module_name().c_str());
    EXPECT_STREQ(expected_result.pose().DebugString().c_str(),
                 localization.pose().DebugString().c_str());
  }

  // TODO(Qi Luo) Update test once got new imu data for euler angle.
}

}  // namespace localization
}  // namespace apollo
