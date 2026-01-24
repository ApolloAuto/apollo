/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/statistics/resource/util.h"

#include <fstream>

namespace apollo {
namespace resource_statistic {

std::vector<std::string> GetStatsLines(const std::string& stat_file, const int line_count) {
    std::vector<std::string> stats_lines;
    std::ifstream buffer(stat_file);
    for (int line_num = 0; line_num < line_count; ++line_num) {
        std::string line;
        std::getline(buffer, line);
        if (line.empty()) {
            break;
        }
        stats_lines.push_back(line);
    }
    return stats_lines;
}

}  // namespace resource_statistic
}  // namespace apollo
