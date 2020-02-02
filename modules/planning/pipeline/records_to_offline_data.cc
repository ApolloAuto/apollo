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
#include "modules/planning/pipeline/feature_generator.h"
#include "modules/prediction/util/data_extraction.h"

DEFINE_string(
    planning_offline_bags, "",
    "a list of bag files or directories for offline mode. The items need to be "
    "separated by colon ':'. ");

namespace apollo {
namespace planning {

void GenerateDataForLearning() {
  const std::vector<std::string> inputs =
      absl::StrSplit(FLAGS_planning_offline_bags, ':');
  FeatureGenerator feature_generator;
  feature_generator.Init();
  for (const auto& input : inputs) {
    std::vector<std::string> offline_bags;
    apollo::prediction::GetRecordFileNames(boost::filesystem::path(input),
                                           &offline_bags);
    std::sort(offline_bags.begin(), offline_bags.end());
    AINFO << "For input " << input << ", found " << offline_bags.size()
          << "  rosbags to process";
    for (std::size_t i = 0; i < offline_bags.size(); ++i) {
      AINFO << "\tProcessing: [ " << i << " / " << offline_bags.size()
            << " ]: " << offline_bags[i];
      feature_generator.ProcessOfflineData(offline_bags[i]);
    }
  }
  feature_generator.Close();
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::planning::GenerateDataForLearning();
  return 0;
}
