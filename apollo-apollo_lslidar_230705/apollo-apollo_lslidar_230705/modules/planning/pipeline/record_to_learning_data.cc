/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <boost/filesystem.hpp>

#include "absl/strings/str_split.h"
#include "cyber/common/file.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/planning/common/feature_output.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/common/util/data_extraction.h"

namespace apollo {
namespace planning {

void GenerateLearningData() {
  AINFO << "map_dir: " << FLAGS_map_dir;
  if (FLAGS_planning_offline_bags.empty()) {
    return;
  }

  if (!FeatureOutput::Ready()) {
    AERROR << "Feature output is not ready.";
    return;
  }

  const std::string planning_config_file =
      "/apollo/modules/planning/conf/planning_config.pb.txt";
  PlanningConfig planning_config;
  ACHECK(
      cyber::common::GetProtoFromFile(planning_config_file, &planning_config))
      << "failed to load planning config file " << planning_config_file;

  MessageProcess message_process;
  if (!message_process.Init(planning_config)) {
    return;
  }

  const std::vector<std::string> inputs =
      absl::StrSplit(FLAGS_planning_offline_bags, ':');
  for (const auto& input : inputs) {
    std::vector<std::string> offline_bags;
    util::GetFilesByPath(boost::filesystem::path(input), &offline_bags);
    std::sort(offline_bags.begin(), offline_bags.end());
    AINFO << "For input " << input << ", found " << offline_bags.size()
          << " rosbags to process";
    for (std::size_t i = 0; i < offline_bags.size(); ++i) {
      AINFO << "\tProcessing: [ " << i + 1 << " / " << offline_bags.size()
            << " ]: " << offline_bags[i];
      message_process.ProcessOfflineData(offline_bags[i]);
      FeatureOutput::WriteRemainderiLearningData(offline_bags[i]);
    }
  }
  message_process.Close();
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::planning::GenerateLearningData();
  return 0;
}
