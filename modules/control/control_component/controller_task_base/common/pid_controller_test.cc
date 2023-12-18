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

#include "modules/control/control_component/controller_task_base/common/pid_controller.h"

#include <iostream>
#include <string>

#include "gtest/gtest.h"

#include "modules/control/control_component/proto/pid_conf.pb.h"

#include "cyber/common/file.h"

namespace apollo {
namespace control {

class PidControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string controllers_dir =
        "/apollo/modules/control/control_component/testdata/conf/";
    std::string station_pid_conf_file =
        controllers_dir + "station_pid_conf.pb.txt";
    std::string speed_pid_conf_file = controllers_dir + "speed_pid_conf.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(station_pid_conf_file,
                                           &station_pid_conf_));
    ACHECK(
        cyber::common::GetProtoFromFile(speed_pid_conf_file, &speed_pid_conf_));
  }

 protected:
  PidConf station_pid_conf_;
  PidConf speed_pid_conf_;
};

TEST_F(PidControllerTest, StationPidController) {
  PidConf pid_conf = station_pid_conf_;
  PIDController pid_controller;
  pid_controller.Init(pid_conf);
  pid_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(0.1, dt), 0.01, 1e-6);
  pid_controller.Reset();
  double control_value = pid_controller.Control(-0.1, dt);
  EXPECT_NEAR(control_value, -0.01, 1e-6);
  dt = 0.0;
  EXPECT_EQ(pid_controller.Control(100, dt), control_value);
  EXPECT_FALSE(pid_controller.IntegratorHold());
}

TEST_F(PidControllerTest, SpeedPidController) {
  PidConf pid_conf = speed_pid_conf_;
  PIDController pid_controller;
  pid_controller.Init(pid_conf);
  pid_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(0.1, dt), 0.1505, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(-0.1, dt), -0.1505, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(500.0, dt), 750.3, 1e-6);
  EXPECT_EQ(pid_controller.IntegratorSaturationStatus(), 1);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(-500.0, dt), -750.3, 1e-6);
  EXPECT_EQ(pid_controller.IntegratorSaturationStatus(), -1);
}

}  // namespace control
}  // namespace apollo
