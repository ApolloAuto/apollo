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

#include "modules/canbus/vehicle/gem/gem_controller.h"

#include <string>

#include "cyber/common/file.h"
#include "gtest/gtest.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/vehicle/gem/gem_message_manager.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

class GemControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string canbus_conf_file =
        "/apollo/modules/canbus/testdata/conf/gem_canbus_conf_test.pb.txt";
    cyber::common::GetProtoFromFile(canbus_conf_file, &canbus_conf_);
    params_ = canbus_conf_.vehicle_parameter();
  }

 protected:
  GemController controller_;
  CanSender<::apollo::canbus::ChassisDetail> sender_;
  CanbusConf canbus_conf_;
  VehicleParameter params_;
  GemMessageManager msg_manager_;
  ControlCommand control_cmd_;
};

TEST_F(GemControllerTest, Init) {
  ErrorCode ret = controller_.Init(params_, &sender_, &msg_manager_);
  EXPECT_EQ(ret, ErrorCode::OK);
}

TEST_F(GemControllerTest, SetDrivingMode) {
  Chassis chassis;
  chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);

  controller_.Init(params_, &sender_, &msg_manager_);

  controller_.set_driving_mode(chassis.driving_mode());
  EXPECT_EQ(controller_.driving_mode(), chassis.driving_mode());
  EXPECT_EQ(controller_.SetDrivingMode(chassis.driving_mode()), ErrorCode::OK);
}

TEST_F(GemControllerTest, Status) {
  controller_.Init(params_, &sender_, &msg_manager_);

  controller_.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  EXPECT_EQ(controller_.Update(control_cmd_), ErrorCode::OK);

  controller_.SetHorn(control_cmd_);
  controller_.SetBeam(control_cmd_);
  controller_.SetTurningSignal(control_cmd_);
  EXPECT_FALSE(controller_.CheckChassisError());
}

TEST_F(GemControllerTest, UpdateDrivingMode) {
  controller_.Init(params_, &sender_, &msg_manager_);

  controller_.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  EXPECT_EQ(controller_.SetDrivingMode(Chassis::COMPLETE_MANUAL),
            ErrorCode::OK);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
