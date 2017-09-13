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

#include <chrono>
#include <thread>

#include "gflags/gflags.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/control/common/control_gflags.h"

DEFINE_string(
    chassis_test_file, "modules/control/testdata/control_tester/chassis.pb.txt",
    "Used for sending simulated Chassis content to the control node.");
DEFINE_string(l10n_test_file,
              "modules/control/testdata/control_tester/localization.pb.txt",
              "Used for sending simulated localization to the control node.");
DEFINE_string(pad_msg_test_file,
              "modules/control/testdata/control_tester/pad_msg.pb.txt",
              "Used for sending simulated PadMsg content to the control node.");
DEFINE_string(
    planning_test_file,
    "modules/control/testdata/control_tester/planning.pb.txt",
    "Used for sending simulated Planning content to the control node.");
DEFINE_int32(num_seconds, 10, "Length of execution.");
DEFINE_int32(feed_frequency, 10,
             "Frequency with which protos are fed to control.");

int main(int argc, char** argv) {
  using std::this_thread::sleep_for;

  using apollo::canbus::Chassis;
  using apollo::common::adapter::AdapterManager;
  using apollo::control::PadMessage;
  using apollo::localization::LocalizationEstimate;
  using apollo::planning::ADCTrajectory;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  const std::string& config_file =
      "modules/control/testdata/control_tester/adapter.conf";
  ros::init(argc, argv, "control_tester");
  AdapterManager::Init(config_file);
  AINFO << "AdapterManager is initialized.";
  for (int i = 0; i < FLAGS_num_seconds * FLAGS_feed_frequency; ++i) {
    AdapterManager::FeedChassisFile(FLAGS_chassis_test_file);
    AdapterManager::FeedLocalizationFile(FLAGS_l10n_test_file);
    AdapterManager::FeedPadFile(FLAGS_pad_msg_test_file);
    AdapterManager::FeedPlanningFile(FLAGS_planning_test_file);
    sleep_for(std::chrono::milliseconds(1000 / FLAGS_feed_frequency));
  }
  AINFO << "Successfully fed proto files.";
  return 0;
}
