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

#include <chrono>
#include <thread>

#include "gflags/gflags.h"
#include "ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/map/relative_map/proto/navigation.pb.h"

DEFINE_string(navigation_dummy_file,
              "modules/map/testdata/navigation_dummy/navigation_info.pb.txt",
              "Used for sending navigation result to relative_map node.");

int main(int argc, char** argv) {
  using std::this_thread::sleep_for;

  using apollo::common::adapter::AdapterManager;
  using apollo::common::adapter::AdapterManagerConfig;
  using apollo::common::adapter::AdapterConfig;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "relative_map_tester");

  FLAGS_alsologtostderr = true;
  AdapterManagerConfig config;
  config.set_is_ros(true);
  auto* sub_config = config.add_config();
  sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
  sub_config->set_type(AdapterConfig::NAVIGATION);

  AdapterManager::Init(config);
  AINFO << "AdapterManager is initialized.";

  apollo::relative_map::NavigationInfo navigation_info;
  if (!apollo::common::util::GetProtoFromFile(FLAGS_navigation_dummy_file,
                                              &navigation_info)) {
    AERROR << "failed to load file: " << FLAGS_navigation_dummy_file;
    return -1;
  }

  ros::Rate loop_rate(0.3);  // frequency

  while (ros::ok()) {
    AdapterManager::FillNavigationHeader("navigation_dummy", &navigation_info);
    AdapterManager::PublishNavigation(navigation_info);
    AINFO << "Sending navigation info:" << navigation_info.DebugString();
    ros::spinOnce();  // yield
    loop_rate.sleep();
  }

  return 0;
}
