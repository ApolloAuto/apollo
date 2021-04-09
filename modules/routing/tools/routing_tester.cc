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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/routing/proto/routing.pb.h"

DEFINE_bool(enable_remove_lane_id, true,
            "True to remove lane id in routing request");

DEFINE_string(routing_test_file,
              "modules/routing/testdata/routing_tester/routing_test.pb.txt",
              "Used for sending routing request to routing node.");

int main(int argc, char** argv) {
  using std::this_thread::sleep_for;

  using apollo::common::adapter::AdapterManager;
  using apollo::common::adapter::AdapterManagerConfig;
  using apollo::common::adapter::AdapterConfig;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "routing_tester");

  FLAGS_alsologtostderr = true;
  AdapterManagerConfig config;
  config.set_is_ros(true);
  auto* sub_config = config.add_config();
  sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
  sub_config->set_type(AdapterConfig::ROUTING_REQUEST);

  AdapterManager::Init(config);
  AINFO << "AdapterManager is initialized.";

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

  ros::Rate loop_rate(0.3);  // frequency

  while (ros::ok()) {
    AdapterManager::FillRoutingRequestHeader("routing", &routing_request);
    AdapterManager::PublishRoutingRequest(routing_request);
    AINFO << "Sending routing request:" << routing_request.DebugString();
    ros::spinOnce();  // yield
    loop_rate.sleep();
  }

  return 0;
}
