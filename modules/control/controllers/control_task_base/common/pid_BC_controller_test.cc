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

#include "modules/control/controllers/control_task_base/common/pid_BC_controller.h"

#include <string>

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/control/controllers/lon_based_pid_controller/proto/lon_based_pid_controller_conf.pb.h"
#include "modules/control/controllers/control_task_base/proto/pid_conf.pb.h"

namespace apollo {
namespace control {

class PidBCControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string controllers_dir = "/apollo/modules/control/controllers/";
    std::string control_conf_file = controllers_dir +
        "lon_based_pid_controller/conf/controller_conf.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(
        control_conf_file, &lon_controller_conf_));
  }

 protected:
  LonBasedPidControllerConf lon_controller_conf_;
};

TEST_F(PidBCControllerTest, StationPidController) {
  PidConf pid_conf = lon_controller_conf_.station_pid_conf();
  PIDBCController pid_BC_controller;
  pid_BC_controller.Init(pid_conf);
  pid_BC_controller.Reset();
  double dt = 1;
  EXPECT_NEAR(pid_BC_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_BC_controller.Reset();
  EXPECT_NEAR(pid_BC_controller.Control(0.1, dt), 0.01, 1e-6);
  pid_BC_controller.Reset();
  double control_value = pid_BC_controller.Control(-0.1, dt);
  EXPECT_NEAR(control_value, -0.01, 1e-6);
  dt = 0.0;
  EXPECT_EQ(pid_BC_controller.Control(100, dt), control_value);
}

TEST_F(PidBCControllerTest, SpeedPidController) {
  PidConf pid_conf = lon_controller_conf_.low_speed_pid_conf();
  PIDBCController pid_BC_controller;
  pid_BC_controller.Init(pid_conf);
  pid_BC_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_BC_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_BC_controller.Reset();
  EXPECT_NEAR(pid_BC_controller.Control(0.1, dt), 0.151, 1e-6);
  pid_BC_controller.Reset();
  EXPECT_NEAR(pid_BC_controller.Control(-0.1, dt), -0.151, 1e-6);
  pid_BC_controller.Reset();
  EXPECT_NEAR(pid_BC_controller.Control(500.0, dt), 3.0, 1e-6);
  EXPECT_EQ(pid_BC_controller.OutputSaturationStatus(), 1);
  pid_BC_controller.Reset();
  EXPECT_NEAR(pid_BC_controller.Control(-500.0, dt), -3.0, 1e-6);
  EXPECT_EQ(pid_BC_controller.OutputSaturationStatus(), -1);
}

}  // namespace control
}  // namespace apollo
