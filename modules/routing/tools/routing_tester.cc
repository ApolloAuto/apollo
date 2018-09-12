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

#include "cybertron/cybertron.h"
#include "cybertron/time/rate.h"
#include "cybertron/time/time.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/routing/proto/routing.pb.h"

DEFINE_bool(enable_remove_lane_id, true,
            "True to remove lane id in routing request");

DEFINE_string(routing_test_file,
              "modules/routing/testdata/routing_tester/routing_test.pb.txt",
              "Used for sending routing request to routing node.");

using apollo::cybertron::Rate;
using apollo::cybertron::Time;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  // init cybertron framework
  apollo::cybertron::Init(argv[0]);
  FLAGS_alsologtostderr = true;

  apollo::routing::RoutingRequest routing_request;
  if (!apollo::common::util::GetProtoFromFile(FLAGS_routing_test_file,
                                              &routing_request)) {
    AERROR << "failed to load file: " << FLAGS_routing_test_file;
    return -1;
  }

  if (FLAGS_enable_remove_lane_id) {
    for (int i = 0; i < routing_request.waypoint_size(); ++i) {
      routing_request.mutable_waypoint(i)->clear_id();
      routing_request.mutable_waypoint(i)->clear_s();
    }
  }

  std::shared_ptr<apollo::cybertron::Node> node(
      apollo::cybertron::CreateNode("routing_tester"));
  auto writer = node->CreateWriter<apollo::routing::RoutingRequest>(
      FLAGS_routing_request_topic);

  Rate rate(1.0);
  while (apollo::cybertron::OK()) {
    writer->Write(routing_request);
    AINFO << "send out routing request: " << routing_request.DebugString();
    rate.Sleep();
  }
  return 0;
}
