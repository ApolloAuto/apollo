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
#include "modules/planning/common/util/util.h"
#include "modules/planning/pipeline/evaluator.h"

DEFINE_string(planning_source_dirs, "",
              "a list of source files or directories for offline mode. "
              "The items need to be separated by colon ':'. ");

namespace apollo {
namespace planning {

void EvaluateTrajectory() {
  const std::vector<std::string> inputs =
      absl::StrSplit(FLAGS_planning_source_dirs, ':');
  Evaluator evaluator;
  evaluator.Init();
  for (const auto& input : inputs) {
    std::vector<std::string> source_files;
    util::GetFilesByPath(boost::filesystem::path(input), &source_files);
    auto it = source_files.begin();
    while (it != source_files.end()) {
      if ((*it).substr((*it).size() - 4) != ".bin") {
        it = source_files.erase(it);
      } else {
        ++it;
      }
    }
    AINFO << "For input " << input << ", found " << source_files.size()
          << " files to process";
    for (std::size_t i = 0; i < source_files.size(); ++i) {
      AINFO << "\tProcessing: [ " << i + 1 << " / " << source_files.size()
            << " ]: " << source_files[i];
      evaluator.Evaluate(source_files[i]);
    }
  }
  evaluator.Close();
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::planning::EvaluateTrajectory();
  return 0;
}
