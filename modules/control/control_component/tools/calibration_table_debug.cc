/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <termios.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/control/control_component/proto/calibration_debug.pb.h"
#include "modules/control/control_component/proto/calibration_table.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/clock.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/control/control_component/common/control_gflags.h"
#include "modules/control/control_component/controller_task_base/common/interpolation_2d.h"

// gflags
DEFINE_string(calibration_table_debug_file,
              "/apollo/modules/control/control_component/conf/"
              "calibration_table.pb.txt",
              "calibration table file");

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::control::DrivingAction;
using apollo::control::PadMessage;
using apollo::cyber::Clock;
using apollo::cyber::CreateNode;
using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class VehicleCalibrationDebug {
 public:
  VehicleCalibrationDebug() : node_(CreateNode("calibration_debug")) {}
  int32_t init() {
    if (is_running_) {
      AERROR << "Already running.";
      return -1;
    }
    is_running_ = true;
    chassis_reader_ = node_->CreateReader<Chassis>(
        FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
          on_chassis(*chassis);
        });

    control_reader_ = node_->CreateReader<ControlCommand>(
        FLAGS_control_command_topic,
        [this](const std::shared_ptr<ControlCommand> &cmd) { on_debug(*cmd); });

    calibration_debug_writer_ = node_->CreateWriter<CalibrationDebug>(
        "/apollo/control/calibration_debug");

    keyboard_thread_.reset(
        new std::thread([this] { KeyboardLoopThreadFunc(); }));
    if (keyboard_thread_ == nullptr) {
      AERROR << "Unable to create can client receiver thread.";
      return -1;
    }

    if (!LoadCalibrationTable(&calibration_table_)) {
      AERROR << "failed to load calibration table";
    }

    InitControlCalibrationTable();

    return 0;
  }

  void Send() {
    apollo::common::util::FillHeader("calibration_debug", &calibration_debug_);
    calibration_debug_writer_->Write(calibration_debug_);
    ADEBUG << "Calibration Debug send OK:"
           << calibration_debug_.ShortDebugString();
  }

  bool LoadCalibrationTable(calibration_table *calibration_table_conf) {
    std::string calibration_table_path = FLAGS_calibration_table_debug_file;

    if (!apollo::cyber::common::GetProtoFromFile(calibration_table_path,
                                                 calibration_table_conf)) {
      AERROR << "Load calibration table failed!";
      return false;
    }
    AINFO << "Load the calibraiton table file successfully, file path: "
          << calibration_table_path;
    return true;
  }

  void InitControlCalibrationTable() {
    AINFO << "Control calibration table size is "
          << calibration_table_.calibration_size();
    Interpolation2D::DataType xyz;
    for (const auto &calibration : calibration_table_.calibration()) {
      xyz.push_back(std::make_tuple(calibration.speed(),
                                    calibration.acceleration(),
                                    calibration.command()));
    }
    control_interpolation_.reset(new Interpolation2D);
    ACHECK(control_interpolation_->Init(xyz))
        << "Fail to load control calibration table";
  }

  void on_debug(const control::ControlCommand &cmd) {
    chassis_reader_->Observe();
    const auto &chassis_msg = chassis_reader_->GetLatestObserved();
    if (chassis_msg == nullptr) {
      AERROR << "Chassis msg is not ready!";
      return;
    }

    double current_speed = 0.0;

    double input_acc_lookup = 0.0;

    double input_throttle_cmd = 0.0;
    double output_throttle_cmd = 0.0;
    double input_brake_cmd = 0.0;
    double output_brake_cmd = 0.0;
    double input_calibration_value = 0.0;
    double output_calibration_value = 0.0;

    if (cmd.debug().simple_lon_debug().has_acceleration_lookup()) {
      current_speed = chassis_msg->speed_mps();
      input_acc_lookup = cmd.debug().simple_lon_debug().acceleration_lookup();
      input_calibration_value =
          cmd.debug().simple_lon_debug().calibration_value();

      output_calibration_value = control_interpolation_->Interpolate(
          std::make_pair(std::fabs(current_speed), input_acc_lookup));

      if (input_acc_lookup * input_calibration_value < 0.0) {
        AINFO << "current_speed:" << current_speed
              << ", input_acc_lookup:" << input_acc_lookup
              << ", input_calibration_value:" << input_calibration_value
              << ", output_calibration_value:" << output_calibration_value;
      }
    }

    Send();
  }

  void on_chassis(const Chassis &chassis) {
    ADEBUG << "Received chassis data: run chassis callback.";
    std::lock_guard<std::mutex> lock(mutex_);
    latest_chassis_.CopyFrom(chassis);
  }

  void KeyboardLoopThreadFunc() {
    char c = 0;
    int32_t kfd_ = 0;
    while (IsRunning()) {
      if (read(kfd_, &c, 1) < 0) {
        perror("read():");
        exit(-1);
      }
    }
  }

  bool IsRunning() const { return is_running_; }

  void stop() {
    if (is_running_) {
      is_running_ = false;
      if (keyboard_thread_ != nullptr && keyboard_thread_->joinable()) {
        keyboard_thread_->join();
        keyboard_thread_.reset();
        AINFO << "keyboard stopped [ok].";
      }
    }
  }

 private:
  std::unique_ptr<std::thread> keyboard_thread_;
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Reader<ControlCommand>> control_reader_;
  std::shared_ptr<Writer<CalibrationDebug>> calibration_debug_writer_;
  std::shared_ptr<Node> node_;

  std::mutex mutex_;

  CalibrationDebug calibration_debug_;
  canbus::Chassis latest_chassis_;
  calibration_table calibration_table_;

  std::unique_ptr<Interpolation2D> control_interpolation_;
  bool is_running_ = false;
};
}  // namespace control
}  // namespace apollo

int main(int argc, char **argv) {
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::control::VehicleCalibrationDebug calibration_debug;
  if (calibration_debug.init() != 0) {
    AERROR << "Fail to init calibration_debug";
    return -1;
  };
  apollo::cyber::WaitForShutdown();
  calibration_debug.stop();
  return 0;
}
