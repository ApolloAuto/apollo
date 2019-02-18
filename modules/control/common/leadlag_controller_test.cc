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

#include "modules/control/common/leadlag_controller.h"

#include <iostream>
#include <string>

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/leadlag_conf.pb.h"

namespace apollo {
namespace control {

class LeadlagControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string control_conf_file =
        "/apollo/modules/control/testdata/conf/control_conf.pb.txt";
    CHECK(cyber::common::GetProtoFromFile(control_conf_file, &control_conf_));
    lon_controller_conf_ = control_conf_.lon_controller_conf();
  }

 protected:
  ControlConf control_conf_;
  LonControllerConf lon_controller_conf_;
};

TEST_F(LeadlagControllerTest, StationLeadlagController) {
  double dt = 0.01;
  LeadlagConf leadlag_conf =
      lon_controller_conf_.reverse_station_leadlag_conf();
  LeadlagController leadlag_controller;
  leadlag_controller.Init(leadlag_conf, dt);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(0.0, dt), 0.0, 1e-6);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(0.1, dt), 0.42, 1e-6);
  leadlag_controller.Reset();
  double control_value = leadlag_controller.Control(-0.1, dt);
  EXPECT_NEAR(control_value, -0.42, 1e-6);
  dt = 0.0;
  EXPECT_EQ(leadlag_controller.Control(100, dt), control_value);
}

TEST_F(LeadlagControllerTest, SpeedLeadlagController) {
  double dt = 0.01;
  LeadlagConf leadlag_conf = lon_controller_conf_.reverse_speed_leadlag_conf();
  LeadlagController leadlag_controller;
  leadlag_controller.Init(leadlag_conf, dt);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(0.0, dt), 0.0, 1e-6);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(0.1, dt), 0.2625, 1e-6);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(-0.1, dt), -0.2625, 1e-6);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(500.0, dt), 6.3, 1e-6);
  EXPECT_EQ(leadlag_controller.InnerstateSaturationStatus(), 1);
  leadlag_controller.Reset();
  EXPECT_NEAR(leadlag_controller.Control(-500.0, dt), -6.3, 1e-6);
  EXPECT_EQ(leadlag_controller.InnerstateSaturationStatus(), -1);
}

}  // namespace control
}  // namespace apollo
