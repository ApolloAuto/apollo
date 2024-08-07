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
#include "modules/common/math/vec2d.h"
namespace apollo {
namespace planning {

class PrintPoints {
 public:
  PrintPoints() {}
  explicit PrintPoints(std::string id) : id_(id) {}
  /**
   * @brief set curve id
   */
  void set_id(std::string id);
  /**
   * @brief add points to curve
   */
  void AddPoint(double x, double y);
  /**
   * @brief print curve to log
   */
  void PrintToLog();

 private:
  std::string id_;
  std::vector<std::pair<double, double>> points;
};

class PrintCurves {
 public:
  /**
   * @brief add point to curve key
   */
  void AddPoint(std::string key, double x, double y);
  void AddPoint(std::string key, const common::math::Vec2d& point);
  void AddPoint(std::string key,
                const std::vector<common::math::Vec2d>& points);
  void PrintToLog();

 private:
  std::map<std::string, PrintPoints> curve_map_;
};
class PrintBox {
 public:
  explicit PrintBox(std::string id) : id_(id) {}
  void AddAdcBox(double x, double y, double heading,
                 bool is_rear_axle_point = true);
  void PrintToLog();

 private:
  std::string id_;
  // x,y, theta, lenth ,width
  std::vector<std::vector<double>> box_points;
};

}  // namespace planning
}  // namespace apollo
