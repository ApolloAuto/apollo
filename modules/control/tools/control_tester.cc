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

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

DEFINE_string(
    chassis_test_file,
    "/apollo/modules/control/testdata/control_tester/chassis.pb.txt",
    "Used for sending simulated Chassis content to the control node.");
DEFINE_string(
    localization_test_file,
    "/apollo/modules/control/testdata/control_tester/localization.pb.txt",
    "Used for sending simulated localization to the control node.");
DEFINE_string(pad_msg_test_file,
              "/apollo/modules/control/testdata/control_tester/pad_msg.pb.txt",
              "Used for sending simulated PadMsg content to the control node.");
DEFINE_string(
    planning_test_file,
    "/apollo/modules/control/testdata/control_tester/planning.pb.txt",
    "Used for sending simulated Planning content to the control node.");
DEFINE_int32(num_seconds, 10, "Length of execution.");
DEFINE_int32(feed_frequency, 10,
             "Frequency with which protos are fed to control.");

int main(int argc, char** argv) {
  using apollo::canbus::Chassis;
  using apollo::control::PadMessage;
  using apollo::cyber::Time;
  using apollo::cyber::common::GetProtoFromFile;
  using apollo::localization::LocalizationEstimate;
  using apollo::planning::ADCTrajectory;
  using std::this_thread::sleep_for;

  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;

  Chassis chassis;
  if (!GetProtoFromFile(FLAGS_chassis_test_file, &chassis)) {
    AERROR << "failed to load file: " << FLAGS_chassis_test_file;
    return -1;
  }

  LocalizationEstimate localization;
  if (!GetProtoFromFile(FLAGS_localization_test_file, &localization)) {
    AERROR << "failed to load file: " << FLAGS_localization_test_file;
    return -1;
  }

  PadMessage pad_msg;
  if (!GetProtoFromFile(FLAGS_pad_msg_test_file, &pad_msg)) {
    AERROR << "failed to load file: " << FLAGS_pad_msg_test_file;
    return -1;
  }

  ADCTrajectory trajectory;
  if (!GetProtoFromFile(FLAGS_planning_test_file, &trajectory)) {
    AERROR << "failed to load file: " << FLAGS_planning_test_file;
    return -1;
  }

  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("control_tester"));
  auto chassis_writer = node->CreateWriter<Chassis>(FLAGS_chassis_topic);
  auto localization_writer =
      node->CreateWriter<LocalizationEstimate>(FLAGS_localization_topic);
  auto pad_msg_writer = node->CreateWriter<PadMessage>(FLAGS_pad_topic);
  auto planning_writer =
      node->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  for (int i = 0; i < FLAGS_num_seconds * FLAGS_feed_frequency; ++i) {
    chassis_writer->Write(chassis);
    localization_writer->Write(localization);
    pad_msg_writer->Write(pad_msg);
    planning_writer->Write(trajectory);
    sleep_for(std::chrono::milliseconds(1000 / FLAGS_feed_frequency));
  }
  AINFO << "Successfully fed proto files.";
  return 0;
}
