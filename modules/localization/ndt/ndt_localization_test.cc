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

#include "modules/localization/ndt/ndt_localization.h"
#include <memory>
#include "cyber/init.h"
#include "gtest/gtest.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace ndt {

class NDTLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    ndt_localization_ptr_.reset(new NDTLocalization());
    apollo::cyber::Init("ndt_localization");
  }
  virtual void TearDown() {}

 protected:
  std::unique_ptr<NDTLocalization> ndt_localization_ptr_;
};

TEST_F(NDTLocalizationTest, Init) {
  FLAGS_local_utm_zone_id = 10;
  FLAGS_online_resolution = 0.25;
  ndt_localization_ptr_->Init();
  EXPECT_EQ(ndt_localization_ptr_->GetZoneId(), 10);
  EXPECT_LE(std::abs(ndt_localization_ptr_->GetOnlineResolution() - 0.25),
            0.01);
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
