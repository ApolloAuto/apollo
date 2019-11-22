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

#include "third_party/json/json.hpp"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/map/relative_map/proto/navigator_config.pb.h"

using apollo::cyber::Rate;
using apollo::relative_map::NavigationInfo;
using apollo::relative_map::NavigationPath;
using apollo::relative_map::NavigatorConfig;
using nlohmann::json;

bool ParseNavigationLineFileNames(
    int argc, char** argv, std::vector<std::string>* navigation_line_filenames);
bool GetNavigationPathFromFile(const std::string& filename,
                               const NavigatorConfig& navigator_config,
                               NavigationPath* navigation_path);
void CheckConfig(const apollo::relative_map::NavigatorConfig& navigator_config);

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Init the cyber framework
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;

  NavigatorConfig navigator_config;
  AINFO << "The navigator configuration filename is: "
        << FLAGS_navigator_config_filename;
  if (!apollo::cyber::common::GetProtoFromFile(FLAGS_navigator_config_filename,
                                               &navigator_config)) {
    AERROR << "Failed to parse " << FLAGS_navigator_config_filename;
    return -1;
  }
  CheckConfig(navigator_config);

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

  ADEBUG << "The flag \"navigator_down_sample\" is: "
         << navigator_config.enable_navigator_downsample();

  NavigationInfo navigation_info;
  int i = 0;
  for (const std::string& filename : navigation_line_filenames) {
    auto* navigation_path = navigation_info.add_navigation_path();
    if (!GetNavigationPathFromFile(filename, navigator_config,
                                   navigation_path)) {
      AWARN << "Failed to load file: " << filename;
      continue;
    }
    AINFO << "The file: " << filename + " is processed ";
    navigation_path->set_path_priority(i);
    navigation_path->mutable_path()->set_name("Navigation path " + i);
    ++i;
  }
  if (navigation_info.navigation_path_size() < 1) {
    AERROR << "No navigation information is fetched.";
    return -1;
  }

  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("navigation_info"));
  auto writer = node->CreateWriter<apollo::relative_map::NavigationInfo>(
      FLAGS_navigation_topic);

  // In theory, the message only needs to be sent once. Considering the problems
  // such as the network delay, We send it several times to ensure that the data
  // is sent successfully.
  Rate rate(1.0);
  static constexpr int kTransNum = 3;
  int trans_num = 0;
  while (apollo::cyber::OK()) {
    if (trans_num > kTransNum) {
      break;
    }
    apollo::common::util::FillHeader(node->Name(), &navigation_info);
    writer->Write(navigation_info);
    ADEBUG << "Sending navigation info:" << navigation_info.DebugString();
    rate.Sleep();
    ++trans_num;
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
                               const NavigatorConfig& navigator_config,
                               NavigationPath* navigation_path) {
  CHECK_NOTNULL(navigation_path);

  std::ifstream ifs(filename, std::ios::in);
  if (!ifs.is_open()) {
    AERROR << "Failed to open " << filename;
    return false;
  }
  std::string line_str;
  double last_sampled_s = std::numeric_limits<double>::min();
  double current_sampled_s = 0.0;
  double diff_s = 0.0;
  double current_kappa = 0.0;
  int original_points_num = 0;
  int down_sampled_points_num = 0;
  const auto& sample_param = navigator_config.sample_param();
  while (std::getline(ifs, line_str)) {
    try {
      auto json_obj = json::parse(line_str);
      current_sampled_s = json_obj["s"];
      current_kappa = json_obj["kappa"];
      diff_s = std::fabs(current_sampled_s - last_sampled_s);
      bool not_down_sampling =
          navigator_config.enable_navigator_downsample()
              ? diff_s >= sample_param.straight_sample_interval() ||
                    (diff_s >= sample_param.small_kappa_sample_interval() &&
                     std::fabs(current_kappa) > sample_param.small_kappa()) ||
                    (diff_s >= sample_param.middle_kappa_sample_interval() &&
                     std::fabs(current_kappa) > sample_param.middle_kappa()) ||
                    (diff_s >= sample_param.large_kappa_sample_interval() &&
                     std::fabs(current_kappa) > sample_param.large_kappa())
              : true;
      // Add a condition: !navigation_path->has_path() to keep the first point
      // when down_sampling
      if (not_down_sampling || !navigation_path->has_path()) {
        last_sampled_s = current_sampled_s;
        auto* point = navigation_path->mutable_path()->add_path_point();
        point->set_x(json_obj["x"]);
        point->set_y(json_obj["y"]);
        point->set_s(current_sampled_s);
        point->set_theta(json_obj["theta"]);
        point->set_kappa(current_kappa);
        point->set_dkappa(json_obj["dkappa"]);
        ++down_sampled_points_num;
        ADEBUG << "down_sample_x: " << json_obj["x"]
               << ", down_sample_y: " << json_obj["y"];
      }
      ++original_points_num;
    } catch (const std::exception& e) {
      AERROR << "Failed to parse JSON data: " << e.what();
      return false;
    }
  }
  AINFO << "The number of original points is: " << original_points_num
        << " and the number of down sampled points is: "
        << down_sampled_points_num << " in the file: " << filename;
  return true;
}

void CheckConfig(
    const apollo::relative_map::NavigatorConfig& navigator_config) {
  CHECK(navigator_config.has_sample_param());
  const auto& sample_param = navigator_config.sample_param();
  CHECK(sample_param.has_straight_sample_interval());
  CHECK(sample_param.has_small_kappa_sample_interval());
  CHECK(sample_param.has_middle_kappa_sample_interval());
  CHECK(sample_param.has_large_kappa_sample_interval());
  CHECK(sample_param.has_small_kappa());
  CHECK(sample_param.has_middle_kappa());
  CHECK(sample_param.has_large_kappa());
}
