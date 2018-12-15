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
 * @brief This file provides the declaration of the class "NavigationMatcher"
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_MATCHER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_MATCHER_H_

#include <string>
#include <vector>

#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/tools/navi_generator/backend/database/db_operator.h"
#include "modules/tools/navi_generator/backend/util/file_operator.h"
#include "third_party/json/json.hpp"

/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

class NavigationMatcher {
 public:
  NavigationMatcher() = default;
  ~NavigationMatcher() = default;

 public:
  bool MatchWayWithPos(const apollo::localization::msf::WGS84Corr position,
                       std::uint64_t* const way_id,
                       std::uint64_t* const line_num,
                       apollo::localization::msf::WGS84Corr* const found_pos);

  bool MatchRoute(const nlohmann::json& expected_route,
                  const std::vector<Way>& route);

 private:
  bool MatchPoint(
      const double x, const double y,
      const std::vector<apollo::planning::ReferencePoint>& lanepoints);

  bool MatchStep(const double x, const double y,
                 const std::vector<std::uint64_t> way_id_vector);

  bool FindMinDist(const std::vector<NaviInfoWithPos>& navi_info,
                   const std::uint64_t data_line_number,
                   const apollo::localization::msf::WGS84Corr position,
                   std::uint64_t* const way_id, std::uint64_t* const line_num,
                   double* const min_dist,
                   apollo::localization::msf::WGS84Corr* const found_pos);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_MATCHER_H_
