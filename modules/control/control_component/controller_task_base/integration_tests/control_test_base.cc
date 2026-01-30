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

#include "modules/control/control_component/controller_task_base/integration_tests/control_test_base.h"

#include <memory>

#include "google/protobuf/text_format.h"

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/util/util.h"
#include "modules/control/control_component/controller_task_base/common/dependency_injector.h"

DEFINE_string(test_chassis_file, "", "chassis input file");
DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_string(test_localization_file, "", "localization input file");
DEFINE_string(test_monitor_file, "", "montor input file");
DEFINE_string(test_pad_file, "", "pad message input file");
DEFINE_string(test_planning_file, "", "planning input file");
DEFINE_bool(test_update_golden_log, false, "true to update golden log file.");

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::monitor::MonitorMessage;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

uint32_t ControlTestBase::s_seq_num_ = 0;

bool ControlTestBase::test_control() {
  if (!cyber::common::GetProtoFromFile(FLAGS_pipeline_file,
                                       &control_.control_pipeline_)) {
    AERROR << "Unable to load control conf file: " << FLAGS_pipeline_file;
    exit(EXIT_FAILURE);
  }

  AINFO << "Pipeline file: " << FLAGS_pipeline_file << " is loaded.";

  // set controller
  control_.injector_ = std::make_shared<DependencyInjector>();

  if (!control_.control_task_agent_
           .Init(control_.injector_, control_.control_pipeline_)
           .ok()) {
    AERROR << "Control init controller failed! Stopping...";
    exit(EXIT_FAILURE);
  }
  control_.control_task_agent_.Reset();

  // Pad message
  if (!FLAGS_test_pad_file.empty()) {
    AINFO << "Into the pad load file.";
    PadMessage pad_message;
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_pad_file, &pad_message)) {
      AERROR << "Failed to load PadMesssage from file " << FLAGS_test_data_dir
             << FLAGS_test_pad_file;
      return false;
    }
    control_.OnPad(std::make_shared<apollo::control::PadMessage>(pad_message));
  }

  // Localization
  if (!FLAGS_test_localization_file.empty()) {
    AINFO << "Into the localization load file.";
    LocalizationEstimate localization;
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_localization_file,
            &localization)) {
      AERROR << "Failed to load localization file " << FLAGS_test_data_dir
             << FLAGS_test_localization_file;
      return false;
    }
    control_.OnLocalization(
        std::make_shared<apollo::localization::LocalizationEstimate>(
            localization));
  }

  // Planning
  if (!FLAGS_test_planning_file.empty()) {
    AINFO << "Into the planning load file.";
    ADCTrajectory trajectory;
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_planning_file, &trajectory)) {
      AERROR << "Failed to load planning file " << FLAGS_test_data_dir
             << FLAGS_test_planning_file;
      return false;
    }
    control_.OnPlanning(
        std::make_shared<apollo::planning::ADCTrajectory>(trajectory));
  }

  // Chassis
  if (!FLAGS_test_chassis_file.empty()) {
    AINFO << "Into the chassis load file.";
    Chassis chassis;
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_chassis_file, &chassis)) {
      AERROR << "Failed to load chassis file " << FLAGS_test_data_dir
             << FLAGS_test_chassis_file;
      return false;
    }
    control_.OnChassis(std::make_shared<apollo::canbus::Chassis>(chassis));
  }

  // Monitor
  if (!FLAGS_test_monitor_file.empty()) {
    AINFO << "Into the monitor load file.";
    MonitorMessage monitor_message;
    if (!cyber::common::GetProtoFromFile(
            FLAGS_test_data_dir + FLAGS_test_monitor_file, &monitor_message)) {
      AERROR << "Failed to load monitor file " << FLAGS_test_data_dir
             << FLAGS_test_monitor_file;
      return false;
    }
    control_.OnMonitor(monitor_message);
  }

  control_.local_view_.mutable_chassis()->CopyFrom(control_.latest_chassis_);
  control_.local_view_.mutable_trajectory()->CopyFrom(
      control_.latest_trajectory_);
  control_.local_view_.mutable_localization()->CopyFrom(
      control_.latest_localization_);

  auto err = control_.ProduceControlCommand(&control_command_);
  if (!err.ok()) {
    ADEBUG << "control ProduceControlCommand failed";
    return false;
  }
  AINFO << "Finish the test_control().";
  return true;
}

void ControlTestBase::trim_control_command(ControlCommand *origin) {
  origin->mutable_header()->clear_radar_timestamp();
  origin->mutable_header()->clear_lidar_timestamp();
  origin->mutable_header()->clear_timestamp_sec();
  origin->mutable_header()->clear_camera_timestamp();
}

bool ControlTestBase::test_control(const std::string &test_case_name,
                                   int case_num) {
  const std::string golden_result_file =
      absl::StrCat("result_", test_case_name, "_", case_num, ".pb.txt");
  std::string tmp_golden_path = "/tmp/" + golden_result_file;
  std::string full_golden_path = FLAGS_test_data_dir + golden_result_file;
  LoadControllerPlugin();
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

void ControlTestBase::LoadControllerPlugin() {
  std::string apollo_root_dir_str = "/apollo";
  // const char* apollo_root_dir = std::getenv("APOLLO_ROOT_DIR");
  // if (apollo_root_dir != nullptr) {
  //   apollo_root_dir_str = std::string(apollo_root_dir);
  // }
  std::string plugin_lib_path =
      "APOLLO_PLUGIN_LIB_PATH=" + apollo_root_dir_str +
      "/bazel-bin:/opt/apollo/neo/lib";
  std::cout << "plugin_lib_path:" << plugin_lib_path << std::endl;
  char lib_path_chars[100];
  memcpy(lib_path_chars, plugin_lib_path.c_str(), plugin_lib_path.size());
  lib_path_chars[plugin_lib_path.size()] = '\0';
  std::cout << "lib_path_chars:" << lib_path_chars << std::endl;
  putenv(lib_path_chars);
  std::string lat_controller_plugin_xml_file =
      "/apollo/modules/control/control_component/testdata/conf/plugins/"
      "lat_based_lqr_controller/plugins.xml";
  std::string lon_controller_plugin_xml_file =
      "/apollo/modules/control/control_component/testdata/conf/plugins/"
      "lon_based_pid_controller/plugins.xml";
  apollo::cyber::plugin_manager::PluginManager::Instance()->LoadPlugin(
      lat_controller_plugin_xml_file);
  apollo::cyber::plugin_manager::PluginManager::Instance()->LoadPlugin(
      lon_controller_plugin_xml_file);
}

void ControlTestBase::SetUpTestCase() {
  FLAGS_pipeline_file =
      "/apollo/modules/control/control_component/testdata/conf/pipeline.pb.txt";
  FLAGS_is_control_test_mode = true;
  FLAGS_is_control_ut_test_mode = true;
  FLAGS_v = 3;
}

void ControlTestBase::SetUp() { ++s_seq_num_; }

}  // namespace control
}  // namespace apollo
