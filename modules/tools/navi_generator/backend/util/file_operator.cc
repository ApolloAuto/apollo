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

/**
 * @file
 * @brief This file provides the implementation of the class
 * "NavigationExpander".
 */
#include "modules/tools/navi_generator/backend/util/file_operator.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace navi_generator {
namespace util {

using apollo::common::util::DistanceXY;
using apollo::planning::ReferencePoint;
using Json = nlohmann::json;

bool FileOperator::Import(
    const std::string& filename,
    std::vector<apollo::planning::ReferencePoint>* const lanepoints) {
  CHECK_NOTNULL(lanepoints);
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    AERROR << "Can't open the smoothed file: " << filename;
    return false;
  }
  std::string line_str;
  double s = 0.0, x = 0.0, y = 0.0, theta = 0.0, kappa = 0.0, dkappa = 0.0;
  while (std::getline(ifs, line_str)) {
    try {
      auto json_obj = Json::parse(line_str);
      kappa = json_obj["kappa"];
      s = json_obj["s"];
      theta = json_obj["theta"];
      x = json_obj["x"];
      y = json_obj["y"];
      dkappa = json_obj["dkappa"];
      ReferencePoint pt;
      pt.set_x(x);
      pt.set_y(y);
      pt.set_heading(theta);
      pt.ToPathPoint(s).set_s(s);
      pt.ToPathPoint(s).set_kappa(kappa);
      pt.ToPathPoint(s).set_dkappa(dkappa);
      lanepoints->push_back(pt);
    } catch (const std::exception& e) {
      AERROR << "Failed to parse JSON data: " << e.what();
      ifs.close();
      return false;
    }
  }
  ifs.close();
  return true;
}

bool FileOperator::Import(const std::string& filename,
                          std::vector<unsigned char>* const data) {
  CHECK_NOTNULL(data);
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    AERROR << "Can't open the smoothed file: " << filename;
    return false;
  }
  // TODO(zhanghua): Maybe these read char operation shuold be optimized.
  unsigned char c;
  while (!ifs.eof()) {
    c = ifs.get();
    data->emplace_back(c);
  }
  ifs.close();
  return true;
}

bool FileOperator::Export(
    const std::string& filename,
    const std::vector<apollo::planning::ReferencePoint>& lanepoints) {
  if (lanepoints.empty()) {
    AERROR << "There aren't any smoothed points to output.";
    return false;
  }

  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) {
    AERROR << "Failed to open the output file: " << filename;
    return false;
  }
  ofs.precision(6);
  double s = 0.0;
  std::size_t size = lanepoints.size();
  // skip the first point and the last point
  for (std::size_t i = 1; i + 1 < size; ++i) {
    const auto& point = lanepoints[i];
    ofs << std::fixed << "{\"kappa\": " << point.kappa() << ", \"s\": " << s
        << ", \"theta\": " << point.heading() << ", \"x\":" << point.x()
        << ", \"y\":" << point.y() << ", \"dkappa\":" << point.dkappa() << "}";
    if (i != size - 2) {
      ofs << '\n';
    }
    s += DistanceXY(point, lanepoints[i + 1]);
  }
  ofs.flush();
  ofs.close();
  AINFO << "The smoothed result is saved to the file: " << filename;
  return true;
}

bool FileOperator::Export(const std::string& filename,
                          const std::vector<unsigned char>& data) {
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) {
    AERROR << "Failed to open the output file: " << filename;
    return false;
  }
  for (auto& c : data) {
    ofs.put(c);
  }
  ofs.flush();
  ofs.close();
  AINFO << "The smoothed result is saved to the file: " << filename;
  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
