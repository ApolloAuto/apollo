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
 * @brief This file provides the declaration of the class
 * "NavigationEditor".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_EDITOR_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_EDITOR_H_

#include <string>
#include <vector>
#include <map>

#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/tools/navi_generator/backend/database/db_operator.h"
#include "modules/tools/navi_generator/backend/util/navigation_matcher.h"
#include "modules/tools/navi_generator/backend/util/trajectory_processor.h"
/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

class NavigationEditor {
 public:
  // NavigationEditor() = default;
  NavigationEditor(UPDATE_GUI_FUNC task, void* gui_service);
  ~NavigationEditor() = default;

 public:
  /**
   * @brief Correct deviation.
   */
  bool CorrectDeviation(
      const std::map<std::uint16_t, FileInfo>& processed_file_info);

  bool CorrectDeviation(const apollo::localization::msf::WGS84Corr& start_point,
                        const apollo::localization::msf::WGS84Corr& end_point);
  /**
   * @brief Correct speed limit.
   */
  bool ModifySpeedLimit(const apollo::localization::msf::WGS84Corr& start_point,
                        const apollo::localization::msf::WGS84Corr& end_point,
                        const std::uint8_t new_speed_min,
                        const std::uint8_t new_speed_max);

  bool SaveRoadCorrection(TrajectoryProcessor* const trajectory_processor);

  bool SaveSpeedLimit();

  bool ResponseToFrontEnd();

  const std::vector<Way>& GetNewRoute() const { return new_route_; }

 private:
  bool FindWayPoint(
      const FileInfo& file_info,
      std::vector<apollo::localization::msf::WGS84Corr>* const waypoints);

  bool GetRightmostNaviFile(const FileInfo& file_info,
                            NaviFile* const navi_file);
  /**
   * @brief Split navigation data by data line number.
   */
  bool SplitNaviData(DBOperator* const db_operator, const std::uint64_t way_id,
                     const std::uint64_t line_number,
                     const Orientation orientation,
                     NaviInfo* const new_navi_info);

  bool SplitWayNodes(DBOperator* const db_operator, const std::uint64_t way_id,
                     const std::uint64_t line_number,
                     const Orientation orientation,
                     WayNodes* const new_way_nodes);

  bool SplitNaviDataByLine(const std::string& file_name,
                           const std::uint64_t line_number,
                           const Orientation orientation,
                           NaviData* const navi_data);
  /**
   * @brief Find route with start and end position.
   */
  bool FindRouteWithPos(DBOperator* const db_operator,
                        const apollo::localization::msf::WGS84Corr& start_point,
                        const apollo::localization::msf::WGS84Corr& end_point,
                        std::uint64_t* const start_line_number,
                        std::uint64_t* const end_line_number,
                        std::vector<Way>* const route);
  /**
   * @brief Update start and end way.
   */
  bool SplitStartEndWay(DBOperator* const db_operator,
                        const std::uint64_t start_way_id,
                        const std::uint64_t end_way_id,
                        const std::uint64_t start_line_number,
                        const std::uint64_t end_line_number);

  bool DeleteOldRoute(DBOperator* const db_operator);

  bool UpdateStartEndWay(DBOperator* const db_operator,
                         const bool is_update_way);
  /**
   * @brief Update new way.
   */
  bool UpdateNewRoute(DBOperator* const db_operator);

 private:
  // first: file_index, second: FileInfo
  std::map<std::uint16_t, FileInfo> processed_file_info_s_;
  std::vector<Way> db_route_;
  std::vector<Way> new_route_;
  NaviInfo start_way_navi_info_;
  NaviInfo end_way_navi_info_;
  WayNodes start_way_nodes_;
  WayNodes end_way_nodes_;
  std::uint8_t new_speed_min_;
  std::uint8_t new_speed_max_;

  // A task for asynchronously updating GUI information.
  std::packaged_task<void(const std::string&)> update_gui_task_;
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_EDITOR_H_
