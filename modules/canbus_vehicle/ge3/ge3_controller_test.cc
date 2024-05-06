/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/ge3/ge3_controller.h"

#include <string>

#include "gtest/gtest.h"
#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus_vehicle/ge3/proto/ge3.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "cyber/common/file.h"
#include "modules/canbus_vehicle/ge3/ge3_message_manager.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"

namespace apollo {
namespace canbus {
namespace ge3 {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

class Ge3ControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string canbus_conf_file =
        "modules/canbus/testdata/conf/ge3_canbus_conf_test.pb.txt";
    cyber::common::GetProtoFromFile(canbus_conf_file, &canbus_conf_);
    params_ = canbus_conf_.vehicle_parameter();
  }

 protected:
  Ge3Controller controller_;
  CanSender<::apollo::canbus::Ge3> sender_;
  CanbusConf canbus_conf_;
  VehicleParameter params_;
  Ge3MessageManager msg_manager_;
  ControlCommand control_cmd_;
};

TEST_F(Ge3ControllerTest, Init) {
  ErrorCode ret = controller_.Init(params_, &sender_, &msg_manager_);
  EXPECT_EQ(ret, ErrorCode::OK);
}

TEST_F(Ge3ControllerTest, SetDrivingMode) {
  Chassis chassis;
  chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);

  controller_.Init(params_, &sender_, &msg_manager_);

  controller_.set_driving_mode(chassis.driving_mode());
  EXPECT_EQ(controller_.driving_mode(), chassis.driving_mode());
  EXPECT_EQ(controller_.SetDrivingMode(chassis.driving_mode()), ErrorCode::OK);
}

TEST_F(Ge3ControllerTest, Status) {
  controller_.Init(params_, &sender_, &msg_manager_);

  controller_.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  EXPECT_EQ(controller_.Update(control_cmd_), ErrorCode::OK);

  controller_.SetHorn(control_cmd_.signal());
  controller_.SetBeam(control_cmd_.signal());
  controller_.SetTurningSignal(control_cmd_.signal());
  EXPECT_FALSE(controller_.CheckChassisError());
}

TEST_F(Ge3ControllerTest, UpdateDrivingMode) {
  controller_.Init(params_, &sender_, &msg_manager_);

  controller_.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  EXPECT_EQ(controller_.SetDrivingMode(Chassis::COMPLETE_MANUAL),
            ErrorCode::OK);
}

}  // namespace ge3
}  // namespace canbus
}  // namespace apollo
