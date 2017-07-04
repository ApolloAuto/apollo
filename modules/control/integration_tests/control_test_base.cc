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

#include <unistd.h>
#include <climits>
#include <fstream>
#include <sstream>

#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/monitor/proto/monitor.pb.h"
#include "modules/common/util/file.h"
#include "modules/control/integration_tests/control_test_base.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "third_party/ros/include/ros/ros.h"

DEFINE_string(test_chassis_file, "", "chassis input file");
DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_string(test_localization_file, "", "localization input file");
DEFINE_string(test_monitor_file, "", "montor input file");
DEFINE_string(test_pad_file, "", "pad message input file");
DEFINE_string(test_planning_file, "", "planning input file");
DEFINE_bool(test_update_golden_log, false,
            "true to update decision golden log file.");

namespace apollo {
namespace control {

using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessage;
using apollo::control::ControlCommand;
using apollo::control::PadMessage;

uint32_t ControlTestBase::_s_seq_num = 0;

bool ControlTestBase::test_control() {
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_control_conf_file,
                                                &control_.control_conf_)) {
    AERROR << "Unable to load control conf file: " << FLAGS_control_conf_file;
    exit(EXIT_FAILURE);
  }

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  // set controller
  if (!control_.controller_agent_.Init(&(control_.control_conf_)).ok()) {
    AERROR << "Control init controller failed! Stopping...";
    exit(EXIT_FAILURE);
  }
  control_.controller_agent_.Reset();

  AdapterManager::Init(FLAGS_adapter_config_path);
  if (!FLAGS_test_pad_file.empty()) {
    PadMessage pad_message;
    apollo::common::util::GetProtoFromFile(
        FLAGS_test_data_dir + FLAGS_test_pad_file, &pad_message);
    control_.OnPad(pad_message);
  }
  if (!FLAGS_test_localization_file.empty()) {
    AdapterManager::FeedLocalizationProtoFile(FLAGS_test_data_dir +
                                              FLAGS_test_localization_file);
  }
  if (!FLAGS_test_planning_file.empty()) {
    AdapterManager::FeedPlanningTrajectoryProtoFile(FLAGS_test_data_dir +
                                                    FLAGS_test_planning_file);
  }
  if (!FLAGS_test_chassis_file.empty()) {
    AdapterManager::FeedChassisProtoFile(FLAGS_test_data_dir +
                                         FLAGS_test_chassis_file);
  }
  if (!FLAGS_test_monitor_file.empty()) {
    MonitorMessage monitor_message;
    apollo::common::util::GetProtoFromFile(
        FLAGS_test_data_dir + FLAGS_test_monitor_file, &monitor_message);
    control_.OnMonitor(monitor_message);
  }

  AdapterManager::Observe();

  auto err = control_.ProduceControlCommand(&control_command_);
  if (!err.ok()) {
    ADEBUG << "control ProduceControlCommand failed";
    return false;
  }
  return true;
}

void ControlTestBase::trim_control_command(ControlCommand* origin) {
  origin->mutable_header()->clear_timestamp_sec();
}

bool ControlTestBase::test_control(const std::string& test_case_name,
                                   int case_num) {
  std::string golden_result_file("result_" + test_case_name + "_" +
                                 std::to_string(case_num) + ".pb.txt");
  std::string tmp_golden_path = "/tmp/" + golden_result_file;
  std::string full_golden_path = FLAGS_test_data_dir + "/" + golden_result_file;
  control_command_.Clear();

  if (!test_control()) {
    AERROR << "test control failed";
    return false;
  }

  trim_control_command(&control_command_);
  if (FLAGS_test_update_golden_log) {
    AINFO << "The golden file is " << tmp_golden_path << " Remember to:\n"
          << "mv " << tmp_golden_path << " " << FLAGS_test_data_dir << "\n"
          << "git add " << FLAGS_test_data_dir << "/" << golden_result_file;
    ::apollo::common::util::SetProtoToASCIIFile(control_command_,
                                                golden_result_file);
  } else {
    ControlCommand golden_result;
    bool load_success = ::apollo::common::util::GetProtoFromASCIIFile(
        full_golden_path, &golden_result);
    if (!load_success) {
      AERROR << "Failed to load golden file: " << full_golden_path;
      ::apollo::common::util::SetProtoToASCIIFile(control_command_,
                                                  tmp_golden_path);
      AINFO << "Current result is written to " << tmp_golden_path;
      return false;
    }
    bool same_result = google::protobuf::util::MessageDifferencer::Equals(
        golden_result, control_command_);
    if (!same_result) {
      std::string tmp_planning_file = tmp_golden_path + ".tmp";
      ::apollo::common::util::SetProtoToASCIIFile(control_command_,
                                                  tmp_planning_file);
      AERROR << "found diff " << tmp_planning_file << " " << full_golden_path;
    }
  }
  return true;
}

void ControlTestBase::SetUpTestCase() {
  ros::Time::init();
  FLAGS_v = 4;
  FLAGS_alsologtostderr = true;
  FLAGS_control_conf_file = "modules/control/testdata/conf/lincoln.pb.txt";
  FLAGS_adapter_config_path = "modules/control/testdata/conf/adapter.conf";
  FLAGS_is_control_test_mode = true;
}

void ControlTestBase::SetUp() { ++_s_seq_num; }

}  // namespace control
}  // namespace apollo
