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

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"

DEFINE_string(routing_dump_file, "/tmp/routing.pb.txt",
              "file name to dump routing response.");

void MessageCallback(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory) {
  if (trajectory->debug().planning_data().has_routing()) {
    std::ofstream dump_file(FLAGS_routing_dump_file);
    dump_file << trajectory->debug().planning_data().routing().DebugString();
    dump_file.close();
    exit(0);
  }
}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;

  auto listener_node = apollo::cyber::CreateNode("routing_dump");
  auto listener = listener_node->CreateReader<apollo::planning::ADCTrajectory>(
      FLAGS_planning_trajectory_topic, MessageCallback);
  apollo::cyber::WaitForShutdown();
  return 0;
}
