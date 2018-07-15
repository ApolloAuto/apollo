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

#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "ros/ros.h"
#include "third_party/json/json.hpp"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/map/relative_map/proto/navigation.pb.h"

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::relative_map::NavigationInfo;
using apollo::relative_map::NavigationPath;
using nlohmann::json;

bool ParseNavigationLineFileNames(
    int argc, char** argv, std::vector<std::string>* navigation_line_filenames);
bool GetNavigationPathFromFile(const std::string& filename,
                               NavigationPath* navigation_path);

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  std::vector<std::string> navigation_line_filenames;
  if (!ParseNavigationLineFileNames(argc, argv, &navigation_line_filenames)) {
    AERROR << "Failed to get navigation file names.";
    AINFO << "Usage: \n"
          << "\t" << argv[0]
          << " navigation_filename_1 navigation_filename_2 ...\n"
          << "For example: \n"
          << "\t" << argv[0]
          << " left.txt.smoothed right.txt.smoothed middle.txt.smoothed";
    return -1;
  }

  ros::init(argc, argv, "navigator_offline", ros::init_options::AnonymousName);

  AdapterManagerConfig config;
  config.set_is_ros(true);
  auto* sub_config = config.add_config();
  sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
  sub_config->set_type(AdapterConfig::NAVIGATION);

  AdapterManager::Init(config);
  ADEBUG << "AdapterManager is initialized.";

  NavigationInfo navigation_info;
  int i = 0;
  for (const std::string& filename : navigation_line_filenames) {
    auto* navigation_path = navigation_info.add_navigation_path();
    if (!GetNavigationPathFromFile(filename, navigation_path)) {
      AWARN << "Failed to load file: " << filename;
      continue;
    }
    AINFO << "Processing " + filename;
    navigation_path->set_path_priority(i);
    navigation_path->mutable_path()->set_name("Navigation path " + i);
    ++i;
  }
  if (navigation_info.navigation_path_size() < 1) {
    AERROR << "No navigation information is fetched.";
    return -1;
  }

  if (ros::ok()) {
    AdapterManager::FillNavigationHeader("relative_map", &navigation_info);
    AdapterManager::PublishNavigation(navigation_info);
    ADEBUG << "Sending navigation info:" << navigation_info.DebugString();

    // Wait for the subscriber's callback function to process this topic.
    // Otherwise, an error message similar to the following will appear:
    // [ERROR] [1530582989.030754209]: Failed to parse message: boost: mutex
    // lock failed in pthread_mutex_lock: Invalid argument
    ros::spinOnce();

    // Sleep for one second to prevent the publishing node from being destroyed
    // prematurely.
    ros::Rate r(1);  // 1 hz
    r.sleep();
  } else {
    AERROR << "ROS status is wrong.";
    return -1;
  }

  return 0;
}

bool ParseNavigationLineFileNames(
    int argc, char** argv,
    std::vector<std::string>* navigation_line_filenames) {
  CHECK_NOTNULL(navigation_line_filenames);
  bool initialized = false;
  if (argc > 1) {
    try {
      navigation_line_filenames->resize(argc - 1);
      for (int i = 0; i < argc - 1; ++i) {
        navigation_line_filenames->at(i) = argv[i + 1];
      }
      initialized = true;
    } catch (const std::exception& e) {
      AERROR << "Failed to get navigation line filenames: " << e.what();
      initialized = false;
    }
  }

  return initialized;
}

bool GetNavigationPathFromFile(const std::string& filename,
                               NavigationPath* navigation_path) {
  CHECK_NOTNULL(navigation_path);

  std::ifstream ifs(filename, std::ios::in);
  if (!ifs.is_open()) {
    AERROR << "Failed to open " << filename;
    return false;
  }
  std::string line_str;
  while (std::getline(ifs, line_str)) {
    try {
      auto json_obj = json::parse(line_str);
      auto* point = navigation_path->mutable_path()->add_path_point();
      point->set_x(json_obj["x"]);
      point->set_y(json_obj["y"]);
      point->set_s(json_obj["s"]);
      point->set_theta(json_obj["theta"]);
      point->set_kappa(json_obj["kappa"]);
      point->set_dkappa(json_obj["dkappa"]);
      ADEBUG << "x: " << json_obj["x"] << ", y: " << json_obj["y"];
    } catch (const std::exception& e) {
      AERROR << "Failed to parse JSON data: " << e.what();
      return false;
    }
  }
  return true;
}
