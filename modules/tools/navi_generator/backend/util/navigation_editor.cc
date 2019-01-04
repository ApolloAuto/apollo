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
 * "NavigationEditor".
 */
#include "modules/tools/navi_generator/backend/util/navigation_editor.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <utility>

#include "google/protobuf/util/json_util.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/tools/navi_generator/backend/util/file_operator.h"
#include "modules/tools/navi_generator/backend/util/trajectory_converter.h"

namespace apollo {
namespace navi_generator {
namespace util {

namespace {
const char kWebSocketTypeCorrectDeviation[] = "requestCorrectRoadDeviation";
const char kWebSocketTypeSaveCorrection[] = "requestSaveRoadCorrection";
const char kWebSocketTypeModifySpeedLmit[] = "requestModifySpeedLimit";
const char kWebSocketTypeSaveSpeedLimit[] = "requestSaveSpeedLimitCorrection";
const size_t kProcessedSuccessfully = 10000;
const size_t kProcessedFailed = 10001;
}  // namespace

using google::protobuf::util::MessageToJsonString;
using std::placeholders::_1;  // for _1, _2, _3...

NavigationEditor::NavigationEditor(UPDATE_FRONTEND_FUNC update_task,
                                   void* gui_service)
    : update_frontend_thread_(std::make_unique<std::thread>(
          &NavigationEditor::UpdateFrontendThread, this)),
      update_frontend_func_(std::bind(update_task, _1, gui_service)),
      update_frontend_finished_(false) {}

NavigationEditor::~NavigationEditor() {
  update_frontend_finished_ = true;
  if (update_frontend_thread_ && update_frontend_thread_->joinable()) {
    update_frontend_thread_->join();
  }
}

bool NavigationEditor::CorrectDeviation(
    const std::map<std::uint16_t, FileInfo>& processed_file_info) {
  std::string response_msg = "Failed to correct road deviation.";
  if (processed_file_info.empty()) {
    ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                       kProcessedFailed);
    return false;
  }
  processed_file_info_s_.clear();
  processed_file_info_s_ = processed_file_info;
  auto item_first = processed_file_info.begin();
  FileInfo file_info_start = item_first->second;
  std::vector<apollo::localization::msf::WGS84Corr> waypoints;
  if (!FindWayPoint(file_info_start, &waypoints)) {
    ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                       kProcessedFailed);
    return false;
  }
  if (waypoints.empty()) {
    ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                       kProcessedFailed);
    return false;
  }
  apollo::localization::msf::WGS84Corr start_point = waypoints.front();
  auto item_last = processed_file_info.rbegin();
  FileInfo file_info_end = item_last->second;
  waypoints.clear();
  if (!FindWayPoint(file_info_end, &waypoints)) {
    ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                       kProcessedFailed);
    return false;
  }
  if (waypoints.empty()) {
    ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                       kProcessedFailed);
    return false;
  }
  apollo::localization::msf::WGS84Corr end_point = waypoints.back();
  start_point_ = start_point;
  end_point_ = end_point;
  if (!CorrectDeviation(start_point, end_point)) {
    ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                       kProcessedFailed);
    return false;
  }
  response_msg = "Corrected road deviation successfully.";
  ResponseToFrontEnd(kWebSocketTypeCorrectDeviation, response_msg,
                     kProcessedSuccessfully);
  return true;
}

bool NavigationEditor::CorrectDeviation(
    const apollo::localization::msf::WGS84Corr& start_point,
    const apollo::localization::msf::WGS84Corr& end_point) {
  DBOperator db_operator;
  std::vector<Way> route;
  std::uint64_t start_line_number = 1, end_line_number = 1;
  std::uint64_t start_way_id = 0, end_way_id = 0;
  need_split_ = true;
  // Find route with start and end position
  if (!FindRouteWithPos(&db_operator, start_point, end_point,
                        &start_line_number, &end_line_number, &route)) {
    AWARN << "Not find route with start and end position.";
    return false;
  }
  if (route.empty()) {
    AWARN << "Founded route is empty.";
    return false;
  }
  // Backup route
  db_route_.clear();
  db_route_ = route;

  start_way_id = route.front().way_id;
  end_way_id = route.back().way_id;
  // Update start and end way
  if (!SplitStartEndWay(&db_operator, start_way_id, end_way_id,
                        start_line_number, end_line_number)) {
    AWARN << "Update start and end way failed.";
    return false;
  }
  return true;
}

bool NavigationEditor::ModifySpeedLimit(
    const apollo::localization::msf::WGS84Corr& start_point,
    const apollo::localization::msf::WGS84Corr& end_point,
    const std::uint8_t new_speed_min, const std::uint8_t new_speed_max) {
  DBOperator db_operator;
  std::vector<Way> route;
  std::uint64_t start_line_number = 1, end_line_number = 1;
  std::uint64_t start_way_id = 0, end_way_id = 0;
  std::string response_msg = "Failed to modify speed limit.";
  // Find route with start and end position
  if (!FindRouteWithPos(&db_operator, start_point, end_point,
                        &start_line_number, &end_line_number, &route)) {
    ResponseToFrontEnd(kWebSocketTypeModifySpeedLmit, response_msg,
                       kProcessedFailed);
    return false;
  }
  if (route.empty()) {
    ResponseToFrontEnd(kWebSocketTypeModifySpeedLmit, response_msg,
                       kProcessedFailed);
    return false;
  }
  // Backup route
  db_route_.clear();
  db_route_ = route;

  start_way_id = route.front().way_id;
  end_way_id = route.back().way_id;

  start_point_ = start_point;
  end_point_ = end_point;
  // Update start and end way
  if (!SplitStartEndWay(&db_operator, start_way_id, end_way_id,
                        start_line_number, end_line_number)) {
    ResponseToFrontEnd(kWebSocketTypeModifySpeedLmit, response_msg,
                       kProcessedFailed);
    return false;
  }
  new_speed_min_ = new_speed_min;
  new_speed_max_ = new_speed_max;

  response_msg = "Modified speed limit successfully.";
  ResponseToFrontEnd(kWebSocketTypeModifySpeedLmit, response_msg,
                     kProcessedSuccessfully);
  return true;
}

bool NavigationEditor::SaveRoadCorrection(
    TrajectoryProcessor* const trajectory_processor) {
  std::string response_msg = "Failed to save road deviation.";
  if (processed_file_info_s_.empty()) {
    ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                       kProcessedFailed);
    return false;
  }
  DBOperator db_operator;
  // Delete old route way
  if (!DeleteOldRoute(&db_operator)) {
    ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                       kProcessedFailed);
    return false;
  }
  // Save new way
  new_route_.clear();
  Way last_way;
  for (auto item = processed_file_info_s_.begin();
       item != processed_file_info_s_.end();) {
    Way way;

    if (!trajectory_processor->SaveWay(&db_operator, last_way, item->second,
                                       &way)) {
      ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                         kProcessedFailed);
      return false;
    }
    if (!trajectory_processor->SaveWayNodes(&db_operator, way.way_id,
                                            item->second)) {
      ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                         kProcessedFailed);
      return false;
    }
    if (!trajectory_processor->SaveWayData(&db_operator, way.way_id,
                                           item->second)) {
      ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                         kProcessedFailed);
      return false;
    }
    if (!trajectory_processor->SaveNaviData(&db_operator, way.way_id,
                                            item->second)) {
      ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                         kProcessedFailed);
      return false;
    }
    last_way = way;
    new_route_.emplace_back(way);
    item = processed_file_info_s_.erase(item);
  }
  // Update start and end way
  if (!UpdateStartEndWay(&db_operator, true)) {
    ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                       kProcessedFailed);
    return false;
  }
  // Update new route
  if (!UpdateNewRoute(&db_operator)) {
    ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                       kProcessedFailed);
    return false;
  }
  response_msg = "Saved road deviation successfully.";
  ResponseToFrontEnd(kWebSocketTypeSaveCorrection, response_msg,
                     kProcessedSuccessfully);
  return true;
}

bool NavigationEditor::SaveSpeedLimit() {
  // Update way speed limit
  std::string response_msg = "Failed to save correction speed limit.";
  DBOperator db_operator;
  for (auto& route_way : db_route_) {
    if ((route_way.way_id == start_way_navi_info_.way_id) ||
        (route_way.way_id == end_way_navi_info_.way_id)) {
      continue;
    }
    if (!db_operator.UpdateWaySpeedLimit(route_way.way_id, new_speed_min_,
                                         new_speed_max_)) {
      ResponseToFrontEnd(kWebSocketTypeSaveSpeedLimit, response_msg,
                         kProcessedSuccessfully);
      return false;
    }
  }
  // Update start and end way
  if (!UpdateStartEndWay(&db_operator, false)) {
    ResponseToFrontEnd(kWebSocketTypeSaveSpeedLimit, response_msg,
                       kProcessedSuccessfully);
    return false;
  }
  response_msg = "Saved correction speed limit successfully.";
  ResponseToFrontEnd(kWebSocketTypeSaveSpeedLimit, response_msg,
                     kProcessedSuccessfully);
  return true;
}

bool NavigationEditor::FindWayPoint(
    const FileInfo& file_info,
    std::vector<apollo::localization::msf::WGS84Corr>* const waypoints) {
  NaviFile rightmost_navi_file;
  if (!GetRightmostNaviFile(file_info, &rightmost_navi_file)) {
    AWARN << "Get right most navigation file failed.";
    return false;
  }
  std::vector<planning::ReferencePoint> smoothed_points;
  FileOperator file_operator;
  if (!file_operator.Import(rightmost_navi_file.navi_file_name,
                            &smoothed_points)) {
    AWARN << "Import file " << rightmost_navi_file.navi_file_name.c_str()
          << "Failed.";
    return false;
  }
  if (smoothed_points.empty()) {
    AWARN << "Not find any smoothed points.";
    return false;
  }

  TrajectoryConverter trajecotry_converter;
  if (!trajecotry_converter.ConvertSmoothedTrajectoryPointsToWGS84(
          &smoothed_points, waypoints)) {
    AWARN << "Convert smoothed points to WGS84 points failed.";
    return false;
  }
  return true;
}

bool NavigationEditor::GetRightmostNaviFile(const FileInfo& file_info,
                                            NaviFile* const navi_file) {
  auto navi_index = 0;
  auto result = std::find_if(std::begin(file_info.navi_files),
                             std::end(file_info.navi_files),
                             [&navi_index](const NaviFile& navi_file_comp) {
                               return navi_file_comp.navi_index == navi_index;
                             });
  if (result == std::end(file_info.navi_files)) {
    return false;
  }
  navi_file->navi_index = result->navi_index;
  navi_file->navi_file_name = result->navi_file_name;
  return true;
}

bool NavigationEditor::SplitNaviData(DBOperator* const db_operator,
                                     const std::uint64_t way_id,
                                     const std::uint64_t line_number,
                                     const Orientation orientation,
                                     NaviInfo* const new_navi_info) {
  CHECK_NOTNULL(new_navi_info);
  // Query way node.
  NaviInfo navi_info;
  navi_info.way_id = way_id;
  new_navi_info->way_id = way_id;

  if (!db_operator->QueryNaviDataWithWayId(way_id, &navi_info.navi_data)) {
    AWARN << "Query navigation data failed.";
    return false;
  }
  // Split data by line number.
  FileOperator file_operator;
  for (std::size_t i = 0; i < navi_info.navi_data.size(); i++) {
    std::string file_name =
        "navi_" + std::to_string(way_id) + std::to_string(line_number) +
        std::to_string(navi_info.navi_data[i].navi_index) + ".smoothed";
    if (!file_operator.Export(file_name, navi_info.navi_data[i].data)) {
      AWARN << "Export navigation data to  " << file_name.c_str() << "failed.";
      return false;
    }
    NaviData navi_data;
    navi_data.navi_index = navi_info.navi_data[i].navi_index;
    if (!SplitNaviDataByLine(file_name, line_number, orientation, &navi_data)) {
      AWARN << "Split navigation data failed.";
      return false;
    }
    if (!navi_data.data.empty()) {
      new_navi_info->navi_data.emplace_back(navi_data);
    }
    // Reomve temporary file
    if (remove(file_name.c_str())) {
      AWARN << "Remove navigation data file " << file_name.c_str()
            << " failed.";
    }
  }
  return true;
}

bool NavigationEditor::SplitWayNodes(DBOperator* const db_operator,
                                     const std::uint64_t way_id,
                                     const std::uint64_t line_number,
                                     const Orientation orientation,
                                     WayNodes* const new_way_nodes) {
  CHECK_NOTNULL(new_way_nodes);
  // Query way node.
  WayNodes way_node;
  if (!db_operator->QueryWayNodesWithWayId(way_id, &way_node)) {
    AWARN << "Query way node failed.";
    return false;
  }
  // Split data by line number.
  for (const auto& node : way_node.nodes) {
    if (orientation == kOrientationForward) {
      if (node.data_line_number > line_number) {
        break;
      }
    } else {
      if (node.data_line_number < line_number) {
        continue;
      }
    }
    new_way_nodes->nodes.emplace_back(node);
  }
  return true;
}

bool NavigationEditor::SplitNaviDataByLine(const std::string& file_name,
                                           const std::uint64_t line_number,
                                           const Orientation orientation,
                                           NaviData* const navi_data) {
  CHECK_NOTNULL(navi_data);
  std::ifstream ifs(file_name.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    AERROR << "Can't open the navigation data file: " << file_name;
    return false;
  }
  std::string str_line;
  std::uint64_t line = 0;
  while (std::getline(ifs, str_line)) {
    line++;
    if (orientation == kOrientationForward) {
      if (line > line_number) {
        break;
      }
    } else {
      if (line < line_number) {
        continue;
      }
    }
    for (std::string::size_type i = 0; i < str_line.length(); i++) {
      navi_data->data.emplace_back(str_line[i]);
    }
  }
  ifs.close();
  return true;
}

bool NavigationEditor::FindRouteWithPos(
    DBOperator* const db_operator,
    const apollo::localization::msf::WGS84Corr& start_point,
    const apollo::localization::msf::WGS84Corr& end_point,
    std::uint64_t* const start_line_number,
    std::uint64_t* const end_line_number, std::vector<Way>* const route) {
  CHECK_NOTNULL(start_line_number);
  CHECK_NOTNULL(end_line_number);
  CHECK_NOTNULL(route);
  CHECK_NOTNULL(db_operator);

  NavigationMatcher navi_matcher;

  std::uint64_t start_way_id = 0, end_way_id = 0;

  apollo::localization::msf::WGS84Corr pos;
  if (!navi_matcher.MatchWayWithPos(start_point, &start_way_id,
                                    start_line_number, &pos)) {
    AWARN << "No match way with start position.";
    return false;
  }
  if (!navi_matcher.MatchWayWithPos(end_point, &end_way_id, end_line_number,
                                    &pos)) {
    AWARN << "No match way with start position.";
    return false;
  }
  // Query route with start and end position
  if (!db_operator->QueryRouteWithStartEndWay(start_way_id, end_way_id,
                                              route)) {
    AWARN << "No route with start and end way.";
    return false;
  }
  // Line number index starting from 1.
  if (*start_line_number < 1) {
    *start_line_number = 1;
  }
  if (*end_line_number < 1) {
    *end_line_number = 1;
  }
  return true;
}

bool NavigationEditor::FindMinMaxLineNumber(
    DBOperator* const db_operator, const std::uint64_t way_id,
    std::uint64_t* const min_line_number,
    std::uint64_t* const max_line_number) {
  // Query way node.
  WayNodes way_node;
  if (!db_operator->QueryWayNodesWithWayId(way_id, &way_node)) {
    AWARN << "Query way node failed.";
    return false;
  }
  *min_line_number = way_node.nodes.front().data_line_number;
  *max_line_number = way_node.nodes.back().data_line_number;
  return true;
}

bool NavigationEditor::SplitStartEndWay(DBOperator* const db_operator,
                                        const std::uint64_t start_way_id,
                                        const std::uint64_t end_way_id,
                                        const std::uint64_t start_line_number,
                                        const std::uint64_t end_line_number) {
  // Check wheter need split
  if (start_way_id == end_way_id) {
    std::uint64_t min_line_number = 0, max_line_number = 0;
    if (!FindMinMaxLineNumber(db_operator, start_way_id, &min_line_number,
                              &max_line_number)) {
      AWARN << "Find way " << start_way_id << " min line number "
            << min_line_number << "max line number " << max_line_number
            << "failed.";
      return false;
    }
    if ((start_line_number == min_line_number) &&
        (end_line_number == max_line_number)) {
      need_split_ = false;
      AINFO << "No need to split way " << start_way_id;
      Way way;
      if (!db_operator->QueryWayWithWayId(start_way_id, &way)) {
        AWARN << "Query start way failed.";
        return false;
      }
      replaced_way_ = way;
      return true;
    }
  }

  // Split start/end navigation data.
  start_way_nodes_.way_id = start_way_id;
  start_way_nodes_.nodes.clear();
  start_way_navi_info_.way_id = start_way_id;
  start_way_navi_info_.navi_data.clear();
  if (!SplitWayNodes(db_operator, start_way_id, start_line_number,
                     kOrientationForward, &start_way_nodes_)) {
    AWARN << "Split start way nodes failed.";
    return false;
  }
  if (!SplitNaviData(db_operator, start_way_id, start_line_number,
                     kOrientationForward, &start_way_navi_info_)) {
    AWARN << "Split start navigation data failed.";
    return false;
  }
  end_way_nodes_.way_id = end_way_id;
  end_way_nodes_.nodes.clear();
  end_way_navi_info_.way_id = end_way_id;
  end_way_navi_info_.navi_data.clear();
  if (!SplitWayNodes(db_operator, end_way_id, end_line_number,
                     kOrientationBackward, &end_way_nodes_)) {
    AWARN << "Split end way nodes failed.";
    return false;
  }
  if (!SplitNaviData(db_operator, end_way_id, end_line_number,
                     kOrientationBackward, &end_way_navi_info_)) {
    AWARN << "Split end navigation data failed.";
    return false;
  }
  return true;
}

bool NavigationEditor::UpdateStartEndWay(DBOperator* const db_operator,
                                         const bool is_update_way) {
  CHECK_NOTNULL(db_operator);
  // Update way nodes
  if (!need_split_) {
    AINFO << "No need to update.";
    return true;
  }

  if (!start_way_nodes_.nodes.empty()) {
    if (!db_operator->UpdateWayNodes(start_way_nodes_.way_id,
                                     start_way_nodes_)) {
      AWARN << "Update start way nodes failed.";
      return false;
    }
  }

  if (!end_way_nodes_.nodes.empty()) {
    if (!db_operator->UpdateWayNodes(end_way_nodes_.way_id, end_way_nodes_)) {
      AWARN << "Update end way nodes failed.";
      return false;
    }
  }
  // Update start/end way navigation data.
  if (!start_way_navi_info_.navi_data.empty()) {
    if (!db_operator->UpdateNaviData(start_way_navi_info_.way_id,
                                     start_way_navi_info_)) {
      AWARN << "Update start way navigation data failed.";
      return false;
    }
  }

  if (!end_way_navi_info_.navi_data.empty()) {
    if (!db_operator->UpdateNaviData(end_way_navi_info_.way_id,
                                     end_way_navi_info_)) {
      AWARN << "Update end way navigation data failed.";
      return false;
    }
  }
  // Is needed update way?
  if (is_update_way) {
    // Query start/end way.
    Way start_way, end_way;
    if (!db_operator->QueryWayWithWayId(start_way_navi_info_.way_id,
                                        &start_way)) {
      AWARN << "Query start way failed.";
      return false;
    }
    if (!db_operator->QueryWayWithWayId(end_way_navi_info_.way_id, &end_way)) {
      AWARN << "Query end way failed.";
      return false;
    }
    // Update start/end way.
    Way new_way_head = new_route_.front();
    Way new_way_tail = new_route_.back();

    start_way.next_way_id = new_way_head.way_id;
    end_way.pre_way_id = new_way_tail.way_id;

    if (!db_operator->UpdateWay(start_way_navi_info_.way_id, start_way)) {
      AWARN << "Update start way failed.";
      return false;
    }
    if (!db_operator->UpdateWay(end_way_navi_info_.way_id, end_way)) {
      AWARN << "Update end way failed.";
      return false;
    }
  }
  AINFO << "Update start end way success.";
  return true;
}

bool NavigationEditor::UpdateNewRoute(DBOperator* const db_operator) {
  // Update new route
  Way new_way_head = new_route_.front();
  Way new_way_tail = new_route_.back();

  if (!need_split_) {
    new_way_head.pre_way_id = replaced_way_.pre_way_id;
    new_way_tail.next_way_id = replaced_way_.next_way_id;
  } else {
    new_way_head.pre_way_id = start_way_navi_info_.way_id;
    new_way_tail.next_way_id = end_way_navi_info_.way_id;
  }

  if (!db_operator->UpdateWay(new_way_head.way_id, new_way_head)) {
    AWARN << "Update new way head failed.";
    return false;
  }
  if (!db_operator->UpdateWay(new_way_tail.way_id, new_way_tail)) {
    AWARN << "Update end way failed.";
    return false;
  }
  // Push start and end way to new route
  if (need_split_) {
    Way start_way, end_way;
    if (!db_operator->QueryWayWithWayId(start_way_navi_info_.way_id,
                                        &start_way)) {
      AWARN << "Query start way failed.";
      return false;
    }
    if (!db_operator->QueryWayWithWayId(end_way_navi_info_.way_id, &end_way)) {
      AWARN << "Query end way failed.";
      return false;
    }
    new_route_.insert(new_route_.begin(), start_way);
    new_route_.emplace_back(end_way);
  }
  AINFO << "Update new route success.";
  return true;
}
bool NavigationEditor::DeleteOldRoute(DBOperator* const db_operator) {
  // Delete between start and end Database route way and then replace new route
  if (db_route_.empty()) {
    AWARN << "Database route is empty.";
    return true;
  }
  for (auto route_way : db_route_) {
    if (need_split_) {
      if ((route_way.way_id == start_way_navi_info_.way_id) ||
          (route_way.way_id == end_way_navi_info_.way_id)) {
        continue;
      }
    }
    if (!db_operator->DeleteWay(route_way.way_id)) {
      AWARN << "Delete way failed.";
      return false;
    }
  }
  return true;
}

bool NavigationEditor::ResponseToFrontEnd(const std::string& type,
                                          const std::string& msg, int success) {
  NaviResponse response;
  response.set_type(type);
  NaviSummary* result = response.mutable_result();
  result->set_success(success);
  result->set_msg(msg);
  if (success == 10000) {
    if (type == kWebSocketTypeCorrectDeviation) {
      // Fill navigation data to response
      NaviRoutePlans* res_data = response.mutable_res_data();
      if (!GetResponseDataByFile(res_data)) {
        AWARN << "Get Response data failed.";
        return false;
      }
    } else if (type == kWebSocketTypeModifySpeedLmit) {
      // Fill navigation data to response
      NaviRoutePlans* res_data = response.mutable_res_data();
      if (!GetResponseDataByDatabase(res_data)) {
        AWARN << "Get Response data failed.";
        return false;
      }
    } else {
      // Nothing to do
    }
  }
  // Inform to frontend.
  std::string response_str;
  if (!MessageToJsonString(response, &response_str).ok()) {
    AWARN << "Navigation response message to json string failed.";
    return false;
  }
  {
    std::lock_guard<std::mutex> update_frontend_lock(update_frontend_mut_);
    update_frontend_tasks_.emplace_back(
        std::bind(update_frontend_func_, response_str));
  }
  update_frontend_cv_.notify_one();
  return true;
}

bool NavigationEditor::GetResponseDataByFile(
    NaviRoutePlans* const response_data) {
  // Set start point.
  NaviWGS84Corr* start_point = response_data->mutable_start();
  start_point->set_lng(start_point_.log);
  start_point->set_lat(start_point_.lat);
  // Set end point.
  NaviWGS84Corr* end_point = response_data->mutable_end();
  end_point->set_lng(end_point_.log);
  end_point->set_lat(end_point_.lat);
  // Set number of plans and plane index.
  response_data->set_num_plans(1);
  // Add route plans.
  NaviRoutePlan* navi_route_plan = response_data->add_route_plans();
  navi_route_plan->set_route_plan_index(0);
  navi_route_plan->set_num_routes(processed_file_info_s_.size());
  std::uint64_t route_index = 0;
  // Add routes into navigation route plans.
  for (auto item = processed_file_info_s_.begin();
       item != processed_file_info_s_.end(); item++) {
    FileInfo file_info = item->second;

    NaviRoute* navi_route = navi_route_plan->add_routes();
    navi_route->set_route_index(route_index);
    navi_route->set_speed_min(file_info.speed_min);
    navi_route->set_speed_max(file_info.speed_max);
    navi_route->set_num_navis(file_info.navi_files.size());
    route_index++;
    // Add navigation path into route.
    for (auto& navi_file : file_info.navi_files) {
      NaviPath* navi_path = navi_route->add_navis();
      navi_path->set_navi_index(navi_file.navi_index);

      std::vector<planning::ReferencePoint> smoothed_points;
      FileOperator file_operator;
      if (!file_operator.Import(navi_file.navi_file_name, &smoothed_points)) {
        continue;
      }
      std::vector<apollo::localization::msf::WGS84Corr> waypoints;
      TrajectoryConverter trajecotry_converter;
      trajecotry_converter.ConvertSmoothedTrajectoryPointsToWGS84(
          &smoothed_points, &waypoints);
      // Add points into navigation path.
      for (const auto& point : waypoints) {
        NaviWGS84Corr* path = navi_path->add_path();
        path->set_lng(point.log);
        path->set_lat(point.lat);
      }
    }
  }
  return true;
}

bool NavigationEditor::GetResponseDataByDatabase(
    NaviRoutePlans* const response_data) {
  // Set start point.
  NaviWGS84Corr* start_point = response_data->mutable_start();
  start_point->set_lng(start_point_.log);
  start_point->set_lat(start_point_.lat);
  // Set end point.
  NaviWGS84Corr* end_point = response_data->mutable_end();
  end_point->set_lng(end_point_.log);
  end_point->set_lat(end_point_.lat);
  // Set number of plans and plane index.
  response_data->set_num_plans(1);
  // Add route plans.
  NaviRoutePlan* navi_route_plan = response_data->add_route_plans();
  navi_route_plan->set_route_plan_index(0);
  navi_route_plan->set_num_routes(db_route_.size());
  // Add routes into navigation route plans.
  DBOperator db_operator;
  for (std::size_t i = 0; i < db_route_.size(); ++i) {
    NaviRoute* navi_route = navi_route_plan->add_routes();
    std::vector<NaviData> navi_data;
    if (!db_operator.QueryNaviDataWithWayId(db_route_[i].way_id, &navi_data)) {
      AWARN << "Failed to query navigation data with way id";
      return false;
    }
    navi_route->set_route_index(i);
    navi_route->set_speed_min(new_speed_min_);
    navi_route->set_speed_max(new_speed_max_);
    navi_route->set_num_navis(navi_data.size());
    // Add navigation path into route.
    for (std::size_t j = 0; j < navi_data.size(); ++j) {
      NaviPath* navi_path = navi_route->add_navis();
      navi_path->set_navi_index(j);
      FileOperator file_operator;
      std::string file_name = "navi_" + std::to_string(j) + ".smoothed";
      if (!file_operator.Export(file_name, navi_data[i].data)) {
        continue;
      }
      std::vector<planning::ReferencePoint> smoothed_points;
      if (!file_operator.Import(file_name, &smoothed_points)) {
        continue;
      }
      std::vector<apollo::localization::msf::WGS84Corr> waypoints;
      TrajectoryConverter trajecotry_converter;
      trajecotry_converter.ConvertSmoothedTrajectoryPointsToWGS84(
          &smoothed_points, &waypoints);
      // Add points into navigation path.
      for (const auto& point : waypoints) {
        NaviWGS84Corr* path = navi_path->add_path();
        path->set_lng(point.log);
        path->set_lat(point.lat);
      }
      // Reomve temporary file
      if (remove(file_name.c_str())) {
        AWARN << "Remove navigation data file " << file_name.c_str()
              << " failed.";
      }
    }
  }
  return true;
}
void NavigationEditor::UpdateFrontendThread() {
  while (!update_frontend_finished_) {
    std::unique_lock<std::mutex> lock(update_frontend_mut_);
    update_frontend_cv_.wait(lock, [this]() {
      return !update_frontend_tasks_.empty() || update_frontend_finished_;
    });
    if (update_frontend_tasks_.empty()) {
      continue;
    }
    auto task = std::move(update_frontend_tasks_.front());
    update_frontend_tasks_.pop_front();

    lock.unlock();
    // Update the frontend display.
    task();
  }
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
