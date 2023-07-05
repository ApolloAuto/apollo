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

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common_msgs/planning_msgs/navigation.pb.h"

using apollo::cyber::Rate;

DEFINE_string(navigation_dummy_file,
              "modules/map/testdata/navigation_dummy/navigation_info.pb.txt",
              "Used for sending navigation result to relative_map node.");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Init the cyber framework
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;

  apollo::relative_map::NavigationInfo navigation_info;
  if (!apollo::cyber::common::GetProtoFromFile(FLAGS_navigation_dummy_file,
                                               &navigation_info)) {
    AERROR << "failed to load file: " << FLAGS_navigation_dummy_file;
    return -1;
  }

  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("navigation_info"));
  auto writer = node->CreateWriter<apollo::relative_map::NavigationInfo>(
      FLAGS_navigation_topic);
  Rate rate(0.3);

  while (apollo::cyber::OK()) {
    apollo::common::util::FillHeader(node->Name(), &navigation_info);
    writer->Write(navigation_info);
    ADEBUG << "Sending navigation info:" << navigation_info.DebugString();
    rate.Sleep();
  }

  return 0;
}
