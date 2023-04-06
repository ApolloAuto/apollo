/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

class PrintPoints {
 public:
    PrintPoints() {}
    explicit PrintPoints(std::string id) : id_(id) {}
    void set_id(std::string id) {
        id_ = id;
    }
    void AddPoint(double x, double y) {
        points.emplace_back(x, y);
    }

    void PrintToLog() {
        std::stringstream ssm;
        ssm << "print_" << id_ << ":";
        for (size_t i = 0; i < points.size(); i++) {
            ssm << std::fixed << "(" << points[i].first << ", "
                << points[i].second << ");";
        }
        AINFO << ssm.str();
    }

 private:
    std::string id_;
    std::vector<std::pair<double, double>> points;
};

class PrintCurves {
 public:
    void AddPoint(std::string key, double x, double y) {
        if (curve_map_.count(key) == 0) {
            curve_map_[key] = PrintPoints(key);
        }
        curve_map_[key].AddPoint(x, y);
    }
     void PrintToLog() {
        for (auto iter = curve_map_.begin(); iter != curve_map_.end();
             iter++ ) {
            iter->second.PrintToLog();
        }
    }
 private:
    std::map<std::string, PrintPoints> curve_map_;
};

}  // namespace planning
}  // namespace apollo
