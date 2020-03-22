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

#include "modules/control/control_component.h"

#include <thread>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "gtest/gtest.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/control_conf.pb.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::monitor::MonitorMessage;
using apollo::common::time::Clock;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

DEFINE_string(test_chassis_file, "", "chassis input file");
DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_string(test_localization_file, "", "localization input file");
DEFINE_string(test_monitor_file, "", "montor input file");
DEFINE_string(test_pad_file, "", "pad message input file");
DEFINE_string(test_planning_file, "", "planning input file");
DEFINE_bool(test_update_golden_log, false, "true to update golden log file.");

class ControlComponentTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_control_conf_file =
        "/apollo/modules/control/testdata/conf/control_conf.pb.txt";
    FLAGS_is_control_test_mode = true;

    SetupCyber();
  }

  virtual void TearDown() {
    if (control_component_) {
      control_component_->Shutdown();
    }
  }

 protected:
  bool FeedTestData();
  void SetupCyber();
  bool RunControl(const std::string& test_case_name);
  void TrimControlCommand(ControlCommand* origin);

 protected:
  bool is_cyber_initialized_ = false;
  std::mutex mutex_;
  cyber::TimerComponentConfig component_config_;

  // cyber readers/writers
  std::shared_ptr<Writer<Chassis>> chassis_writer_;
  std::shared_ptr<Writer<LocalizationEstimate>> localization_writer_;
  std::shared_ptr<Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<Writer<PadMessage>> pad_writer_;
  std::shared_ptr<Reader<ControlCommand>> control_reader_;

  std::shared_ptr<ControlComponent> control_component_ = nullptr;
  ControlCommand control_command_;
  MonitorMessage monitor_message_;
  Chassis chassis_;
  ADCTrajectory trajectory_;
  LocalizationEstimate localization_;
  PadMessage pad_message_;
};

void ControlComponentTest::SetupCyber() {
  if (is_cyber_initialized_) {
    return;
  }

  // init cyber framework
  apollo::cyber::Init("control_test");

  Clock::SetMode(Clock::CYBER);

  component_config_.set_name("control_test");

  component_config_.set_interval(10);

  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("control_test"));

  chassis_writer_ = node->CreateWriter<Chassis>(FLAGS_chassis_topic);
  localization_writer_ =
      node->CreateWriter<LocalizationEstimate>(FLAGS_localization_topic);
  pad_writer_ = node->CreateWriter<PadMessage>(FLAGS_pad_topic);
  planning_writer_ =
      node->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  control_reader_ = node->CreateReader<ControlCommand>(
      FLAGS_control_command_topic,
      [this](const std::shared_ptr<ControlCommand>& control_command) {
        ADEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_command_.CopyFrom(*control_command);
      });

  is_cyber_initialized_ = true;
}

bool ControlComponentTest::FeedTestData() {
  // Pad message
  if (!FLAGS_test_pad_file.empty()) {
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_pad_file, &pad_message_)) {
      AERROR << "Failed to load PadMesssage from file " << FLAGS_test_data_dir
             << FLAGS_test_pad_file;
      return false;
    }
  }

  // Localization
  if (!FLAGS_test_localization_file.empty()) {
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_localization_file,
            &localization_)) {
      AERROR << "Failed to load localization file " << FLAGS_test_data_dir
             << FLAGS_test_localization_file;
      return false;
    }
  }

  // Planning
  if (!FLAGS_test_planning_file.empty()) {
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_planning_file, &trajectory_)) {
      AERROR << "Failed to load planning file " << FLAGS_test_data_dir
             << FLAGS_test_planning_file;
      return false;
    }
  }

  // Chassis
  if (!FLAGS_test_chassis_file.empty()) {
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_chassis_file, &chassis_)) {
      AERROR << "Failed to load chassis file " << FLAGS_test_data_dir
             << FLAGS_test_chassis_file;
      return false;
    }
  }

  // Monitor
  if (!FLAGS_test_monitor_file.empty()) {
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_monitor_file, &monitor_message_)) {
      AERROR << "Failed to load monitor file " << FLAGS_test_data_dir
             << FLAGS_test_monitor_file;
      return false;
    }
  }
  AINFO << "Successfully feed proto files.";
  return true;
}

bool ControlComponentTest::RunControl(const std::string& test_case_name) {
  ACHECK(FeedTestData()) << "Failed to feed test data";

  control_component_.reset(new ControlComponent());
  control_component_->Initialize(component_config_);

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // feed topics
  planning_writer_->Write(trajectory_);
  chassis_writer_->Write(chassis_);
  localization_writer_->Write(localization_);
  chassis_writer_->Write(chassis_);
  pad_writer_->Write(pad_message_);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  TrimControlCommand(&control_command_);

  const std::string golden_result_file =
      absl::StrCat("result_", test_case_name, ".pb.txt");
  std::string tmp_golden_path = "/tmp/" + golden_result_file;
  std::string full_golden_path = FLAGS_test_data_dir + golden_result_file;
  control_command_.Clear();

  if (FLAGS_test_update_golden_log) {
    AINFO << "The golden file is " << tmp_golden_path << " Remember to:\n"
          << "mv " << tmp_golden_path << " " << FLAGS_test_data_dir << "\n"
          << "git add " << FLAGS_test_data_dir << "/" << golden_result_file;
    cyber::common::SetProtoToASCIIFile(control_command_, golden_result_file);
  } else {
    ControlCommand golden_result;
    bool load_success =
        cyber::common::GetProtoFromASCIIFile(full_golden_path, &golden_result);
    if (!load_success) {
      AERROR << "Failed to load golden file: " << full_golden_path;
      cyber::common::SetProtoToASCIIFile(control_command_, tmp_golden_path);
      AINFO << "Current result is written to " << tmp_golden_path;
      return false;
    }
    bool same_result =
        common::util::IsProtoEqual(golden_result, control_command_);
    if (!same_result) {
      std::string tmp_test_result_file = tmp_golden_path + ".tmp";
      cyber::common::SetProtoToASCIIFile(control_command_,
                                         tmp_test_result_file);
      AERROR << "found diff " << tmp_test_result_file << " "
             << full_golden_path;
    }
  }
  return true;
}

void ControlComponentTest::TrimControlCommand(ControlCommand* origin) {
  origin->mutable_header()->clear_radar_timestamp();
  origin->mutable_header()->clear_lidar_timestamp();
  origin->mutable_header()->clear_timestamp_sec();
  origin->mutable_header()->clear_camera_timestamp();
}

TEST_F(ControlComponentTest, simple_test) {
  FLAGS_test_data_dir = "/apollo/modules/control/testdata/simple_control_test/";
  FLAGS_enable_csv_debug = true;
  FLAGS_test_localization_file = "1_localization.pb.txt";
  FLAGS_test_pad_file = "1_pad.pb.txt";
  FLAGS_test_planning_file = "1_planning.pb.txt";
  FLAGS_test_chassis_file = "1_chassis.pb.txt";
  bool run_control_success = RunControl("simple_test_0");
  EXPECT_TRUE(run_control_success);
}

}  // namespace control
}  // namespace apollo
