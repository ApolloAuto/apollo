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
 * "TrajectoryProcessor".
 */

#include "modules/tools/navi_generator/backend/util/trajectory_processor.h"

#include <fstream>
#include <functional>
#include <list>
#include <utility>

#include "google/protobuf/util/json_util.h"
#include "modules/common/util/util.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"
#include "modules/tools/navi_generator/backend/util/file_operator.h"
#include "modules/tools/navi_generator/backend/util/navigation_expander.h"
#include "modules/tools/navi_generator/backend/util/quad_tiles_maker.h"

namespace apollo {
namespace navi_generator {
namespace util {

namespace {
constexpr double kOverlapLenFromSecondBag = 250.0;
constexpr double kDefaultLaneWidth = 3.75;
const char kWebSocketTypeProcessFile[] = "requestProcessBagFiles";
const char kWebSocketTypeSaveFile[] = "requestSaveBagFiles";
const size_t kProcessedSuccessfully = 0;
const size_t kProcessedFailed = 1;
const size_t kAllFilesProcessedSuccessfully = 10000;
const size_t kLastFileProcessedFailed = 10001;
}  // namespace

using google::protobuf::util::MessageToJsonString;
using std::placeholders::_1;  // for _1, _2, _3...

TrajectoryProcessor::TrajectoryProcessor(
    UPDATE_FRONTEND_FUNC update_task,
    NOTIFY_BAG_FIFES_PROCESSED_FUNC notify_task, void* gui_service)
    : file_seg_thread_(std::make_unique<std::thread>(
          &TrajectoryProcessor::FileSegmentThread, this)),
      file_seg_finished_(false),
      update_frontend_thread_(std::make_unique<std::thread>(
          &TrajectoryProcessor::UpdateFrontendThread, this)),
      update_frontend_func_(std::bind(update_task, _1, gui_service)),
      update_frontend_finished_(false),
      notify_bag_file_processed_func_(std::bind(notify_task, gui_service)) {}

TrajectoryProcessor::~TrajectoryProcessor() {
  file_seg_finished_ = true;
  if (file_seg_thread_ && file_seg_thread_->joinable()) {
    file_seg_thread_->join();
  }

  update_frontend_finished_ = true;
  if (update_frontend_thread_ && update_frontend_thread_->joinable()) {
    update_frontend_thread_->join();
  }
}

void TrajectoryProcessor::FileSegmentThread() {
  while (!file_seg_finished_) {
    std::unique_lock<std::mutex> file_seg_lock(file_seg_mut_);
    file_seg_cv_.wait(file_seg_lock, [this]() {
      return !cur_file_segment_s_.empty() || file_seg_finished_;
    });
    if (!cur_file_segment_s_.empty() &&
        cur_file_segment_s_[0].cur_file_name.empty()) {
      cur_file_segment_s_.clear();
      continue;
    }
    if (!cur_file_segment_s_.empty() &&
        cur_file_segment_s_.size() <
            cur_file_segment_s_[0].cur_file_seg_count) {
      continue;
    }
    // All segments of a file have been received.
    if (!cur_file_segment_s_.empty()) {
      std::uint16_t file_index = cur_file_segment_s_[0].cur_file_index;
      std::string cur_file_name = cur_file_segment_s_[0].cur_file_name;
      std::string next_file_name = cur_file_segment_s_[0].next_file_name;
      if (!cur_file_name.empty() && !ExportSegmentsToFile(cur_file_name)) {
        cur_file_segment_s_.clear();
        file_seg_lock.unlock();

        // Inform the frontend an error message.
        NaviResponse response;
        response.set_type(kWebSocketTypeProcessFile);
        NaviSummary* result = response.mutable_result();
        result->set_success(kProcessedFailed);
        result->set_msg("Failed to store received file " + cur_file_name);
        ResponseToFrontend(response);
        continue;
      }
      cur_file_segment_s_.clear();
      file_seg_lock.unlock();

      BagFileInfo bag_file_info;
      bag_file_info.file_index = file_index;
      bag_file_info.raw_file_name = cur_file_name;
      bag_file_info.next_raw_file_name = next_file_name;
      bag_files_to_process_[bag_file_info.file_index] = bag_file_info;
    }

    if (!ProcessFiles()) {
      continue;
    }
  }
}

void TrajectoryProcessor::UpdateFrontendThread() {
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

bool TrajectoryProcessor::SetCommonBagFileInfo(
    const CommonBagFileInfo& common_file_info) {
  common_file_info_ = common_file_info;
  return true;
}

bool TrajectoryProcessor::ProcessBagFileSegment(
    const FileSegment& file_segment) {
  {
    std::lock_guard<std::mutex> lk(file_seg_mut_);
    cur_file_segment_s_[file_segment.cur_file_seg_index] = file_segment;
    if (cur_file_segment_s_.size() < file_segment.cur_file_seg_count) {
      return true;
    }
  }
  file_seg_cv_.notify_one();
  return true;
}

bool TrajectoryProcessor::SaveFilesToDatabase() {
  DBOperator db_operator;
  Way last_way;
  bool res = true;
  for (auto item = processed_file_info_s_.begin();
       item != processed_file_info_s_.end();) {
    Way way;
    if (!SaveWay(&db_operator, last_way, item->second, &way)) {
      res = false;
      break;
    }

    if (!SaveWayNodes(&db_operator, way.way_id, item->second)) {
      res = false;
      break;
    }

    if (!SaveWayData(&db_operator, way.way_id, item->second)) {
      res = false;
      break;
    }

    if (!SaveNaviData(&db_operator, way.way_id, item->second)) {
      res = false;
      break;
    }

    last_way = way;
    item = processed_file_info_s_.erase(item);
  }

  NaviResponse response;
  NaviSummary* result = response.mutable_result();
  if (res) {
    // Inform the frontend the result.
    response.set_type(kWebSocketTypeSaveFile);
    result->set_success(kProcessedSuccessfully);
    result->set_msg("The bags has been saved successfully");
  } else {
    // Inform the frontend an error message.
    response.set_type(kWebSocketTypeSaveFile);
    result->set_success(kProcessedFailed);
    result->set_msg("Failed to save bags files to database");
  }
  ResponseToFrontend(response);
  return res;
}

bool TrajectoryProcessor::SaveWay(DBOperator* const db_operator,
                                  const Way& last_way,
                                  const FileInfo& file_info, Way* const way) {
  if (file_info.bag_file_info.file_index == 0) {
    if (!db_operator->CreateNewWayId(&(way->way_id))) {
      return false;
    }
    way->pre_way_id = 0;
  } else {
    way->way_id = last_way.way_id + 1;
    way->pre_way_id = (last_way.next_way_id == 0) ? 0 : last_way.way_id;
  }
  way->next_way_id = file_info.bag_file_info.next_raw_file_name.empty()
                         ? 0
                         : (way->way_id + 1);
  way->speed_min = file_info.speed_min;
  way->speed_max = file_info.speed_max;
  if (!db_operator->SaveWay(*way)) {
    return false;
  }
  return true;
}

bool TrajectoryProcessor::SaveWayNodes(DBOperator* const db_operator,
                                       const std::uint64_t way_id,
                                       const FileInfo& file_info) {
  NaviFile rightmost_navi_file;
  if (!GetRightmostNaviFile(file_info, &rightmost_navi_file)) {
    return false;
  }
  std::vector<planning::ReferencePoint> smoothed_points;
  FileOperator file_operator;
  if (!file_operator.Import(rightmost_navi_file.navi_file_name,
                            &smoothed_points)) {
    return false;
  }
  std::vector<apollo::localization::msf::WGS84Corr> waypoints;
  TrajectoryConverter trajecotry_converter;
  trajecotry_converter.ConvertSmoothedTrajectoryPointsToWGS84(&smoothed_points,
                                                              &waypoints);
  WayNodes way_nodes;
  if (!FindWayNodes(way_id, &waypoints, &way_nodes)) {
    return false;
  }
  if (!db_operator->SaveWayNodes(way_nodes)) {
    return false;
  }
  return true;
}

bool TrajectoryProcessor::SaveWayData(DBOperator* const db_operator,
                                      const std::uint64_t way_id,
                                      const FileInfo& file_info) {
  WayData way_data;
  way_data.way_id = way_id;
  std::vector<unsigned char> raw_data;
  FileOperator file_operator;
  if (!file_operator.Import(file_info.bag_file_info.raw_file_name, &raw_data)) {
    AERROR << "import failed.";
    return false;
  }
  way_data.raw_data = raw_data;
  way_data.navi_number =
      common_file_info_.left_lane_num + common_file_info_.right_lane_num + 1;
  // TODO(*): save the navi_data in correct db table when saving in sub table
  way_data.navi_table_id = 0;
  if (!db_operator->SaveWayData(way_data)) {
    return false;
  }
  return true;
}

bool TrajectoryProcessor::SaveNaviData(DBOperator* const db_operator,
                                       const std::uint64_t way_id,
                                       const FileInfo& file_info) {
  NaviInfo navi_info;
  navi_info.way_id = way_id;
  std::vector<NaviData> navi_data;
  for (auto& navi_file : file_info.navi_files) {
    NaviData navi;
    navi.navi_index = navi_file.navi_index;
    std::vector<unsigned char> data;
    FileOperator file_operator;
    if (!file_operator.Import(navi_file.navi_file_name, &data)) {
      return false;
    }
    navi.data = data;
    navi_data.emplace_back(navi);
  }
  navi_info.navi_data = navi_data;
  if (!db_operator->SaveNaviData(navi_info)) {
    return false;
  }
  return true;
}

bool TrajectoryProcessor::Reset() {
  cur_file_segment_s_.clear();
  processed_file_info_s_.clear();
  return true;
}

bool TrajectoryProcessor::ExportSegmentsToFile(const std::string& file_name) {
  if (cur_file_segment_s_.size() < 1) {
    AERROR << "There aren't any file segment to output.";
    return false;
  }
  std::ofstream ofs(file_name.c_str());
  if (!ofs.is_open()) {
    AERROR << "Failed to open the output file: " << file_name;
    return false;
  }
  FileSegment segment;
  for (std::uint16_t i = 0; i < cur_file_segment_s_.size(); ++i) {
    ofs << apollo::common::util::DecodeBase64(
        cur_file_segment_s_[i].cur_file_seg_data);
  }
  ofs.close();

  return true;
}

bool TrajectoryProcessor::ProcessFiles() {
  BagFileInfo bag_file_info;
  while (bag_files_to_process_.size() > 0) {
    auto item = bag_files_to_process_.begin();
    bag_file_info = item->second;
    std::string next_file_to_process;
    if (bag_file_info.file_index + 1 == common_file_info_.total_file_num) {
      // the last file to process
      if (!ProcessFile(bag_file_info)) {
        // Inform the frontend an error message.
        NaviResponse response;
        response.set_type(kWebSocketTypeProcessFile);
        NaviSummary* result = response.mutable_result();
        result->set_success(kLastFileProcessedFailed);
        result->set_msg("Failed to process file" + bag_file_info.raw_file_name);
        ResponseToFrontend(response);
        bag_files_to_process_.erase(bag_file_info.file_index);
        return false;
      } else {
        notify_bag_file_processed_func_();
        bag_files_to_process_.erase(bag_file_info.file_index);
      }
      break;
    }
    // the 1st~ (total_file_num-1)th file to process
    if (++item != bag_files_to_process_.end()) {
      if (!ProcessFile(bag_file_info)) {
        // Inform the frontend an error message.
        NaviResponse response;
        response.set_type(kWebSocketTypeProcessFile);
        NaviSummary* result = response.mutable_result();
        result->set_success(kProcessedFailed);
        result->set_msg("Failed to process file" + bag_file_info.raw_file_name);
        ResponseToFrontend(response);
        bag_files_to_process_.erase(bag_file_info.file_index);
        return false;
      } else {
        bag_files_to_process_.erase(bag_file_info.file_index);
      }
    } else {
      break;
    }
  }
  return true;
}

bool TrajectoryProcessor::ProcessFile(const BagFileInfo& bag_file_info) {
  double speed_min;
  double speed_max;
  TrajectoryConverter trajectory_converter;
  if (!trajectory_converter.ExtractSpeedLimit(bag_file_info.raw_file_name,
                                              &speed_min, &speed_max)) {
    return false;
  }

  wgs84_points_.clear();
  std::string smoothed_file_name;
  if (!ProcessBagFile(bag_file_info.raw_file_name,
                      bag_file_info.next_raw_file_name, &smoothed_file_name,
                      &wgs84_points_)) {
    return false;
  }
  // expand the lane
  std::vector<NaviFile> navi_files;
  if (!ExpandNaviFiles(smoothed_file_name, &navi_files)) {
    return false;
  }
  FileInfo file_info;
  file_info.bag_file_info = bag_file_info;
  file_info.navi_files = navi_files;
  file_info.speed_min = static_cast<uint8_t>(speed_min);
  file_info.speed_max = static_cast<uint8_t>(speed_max);

  // Inform the frontend the result.
  {
    NaviResponse response;
    response.set_type(kWebSocketTypeProcessFile);
    NaviSummary* result = response.mutable_result();
    result->set_success(
        (processed_file_info_s_.size() + 1 >= common_file_info_.total_file_num)
            ? kAllFilesProcessedSuccessfully
            : kProcessedSuccessfully);
    result->set_msg("Processed file " + bag_file_info.raw_file_name +
                    " successfully");

    NaviRoutePlans* res_data = response.mutable_res_data();
    NaviWGS84Corr* start_corr = res_data->mutable_start();
    NaviWGS84Corr* end_corr = res_data->mutable_end();
    res_data->set_num_plans(1);
    NaviRoutePlan* route_plans = res_data->add_route_plans();
    route_plans->set_num_routes(1);
    route_plans->set_route_plan_index(0);
    NaviRoute* routes = route_plans->add_routes();
    for (auto& navi_file : file_info.navi_files) {
      NaviPath* navis = routes->add_navis();
      navis->set_navi_index(navi_file.navi_index);
      std::vector<planning::ReferencePoint> smoothed_points;
      FileOperator file_operator;
      if (!file_operator.Import(navi_file.navi_file_name, &smoothed_points)) {
        break;
      }
      std::vector<apollo::localization::msf::WGS84Corr> waypoints;
      TrajectoryConverter trajecotry_converter;
      trajecotry_converter.ConvertSmoothedTrajectoryPointsToWGS84(
          &smoothed_points, &waypoints);
      for (const auto& point : waypoints) {
        NaviWGS84Corr* path = navis->add_path();
        path->set_lng(point.log);
        path->set_lat(point.lat);
      }

      if (navi_file.navi_index == 0) {
        apollo::localization::msf::WGS84Corr start = waypoints.front();
        apollo::localization::msf::WGS84Corr end = waypoints.back();
        start_corr->set_lng(start.log);
        start_corr->set_lat(start.lat);
        end_corr->set_lng(end.log);
        end_corr->set_lat(end.lat);
      }
    }
    routes->set_num_navis(routes->navis().size());
    routes->set_route_index(0);
    routes->set_speed_min(file_info.speed_min);
    routes->set_speed_max(file_info.speed_max);
    ResponseToFrontend(response);
  }

  processed_file_info_s_.insert(
      std::make_pair(file_info.bag_file_info.file_index, file_info));
  return true;
}

bool TrajectoryProcessor::ProcessBagFile(
    const std::string& first_bag_filename,
    const std::string& second_bag_filename,
    std::string* const smoothed_file_name,
    std::vector<apollo::localization::msf::WGS84Corr>* const waypoints) {
  CHECK_NOTNULL(waypoints);
  TrajectoryConverter trajectory_converter;
  if (!trajectory_converter.ExtractTrajectoryPointsFromTwoBags(
          first_bag_filename, second_bag_filename, kOverlapLenFromSecondBag)) {
    return false;
  }

  if (!trajectory_converter.SmoothTrajectoryPoints()) {
    return false;
  }

  if (!trajectory_converter.SaveSmoothedTrajectoryPoints()) {
    return false;
  }
  *smoothed_file_name = trajectory_converter.GetSmoothedFileName();

  if (!trajectory_converter.GetSmoothedTrajectoryWGS84Points(waypoints)) {
    return false;
  }

  return true;
}

bool TrajectoryProcessor::ExpandNaviFiles(
    const std::string& src_smoothed_file_name,
    std::vector<NaviFile>* const navi_files) {
  std::list<ExpandedFileInfo> expanded_files;
  NavigationExpander navigation_expander;
  if (!navigation_expander.ExpandLane(src_smoothed_file_name,
                                      common_file_info_.left_lane_num,
                                      common_file_info_.right_lane_num,
                                      kDefaultLaneWidth, &expanded_files)) {
    return false;
  }
  for (auto& item : expanded_files) {
    NaviFile navi_file;
    navi_file.navi_index = item.index;
    navi_file.navi_file_name = item.file_name;
    navi_files->emplace_back(navi_file);
  }
  return true;
}

bool TrajectoryProcessor::FindWayNodes(
    const std::uint64_t way_id,
    const std::vector<apollo::localization::msf::WGS84Corr>* const waypoints,
    WayNodes* const way_nodes) {
  CHECK_NOTNULL(waypoints);
  QuadTilesMaker tile_maker;
  QuadTile quad_tile;
  std::uint64_t node_index = 0;
  std::uint64_t line_number = 0;
  std::string node_value;
  std::vector<Node> nodes;
  way_nodes->way_id = way_id;
  for (auto& waypoint : *waypoints) {
    ++line_number;
    tile_maker.MakeQuadTile(waypoint.lat, waypoint.log, -1,
                            FLAGS_quadtile_zoom_level, &quad_tile);
    tile_maker.IdAsString(FLAGS_quadtile_zoom_level, &quad_tile, &node_value);
    Node node = {node_index, line_number, node_value};
    auto node_itr = std::find_if(
        nodes.begin(), nodes.end(),
        [&node](const Node& obj) { return obj.node_value == node.node_value; });
    if (node_itr == nodes.end()) {
      nodes.emplace_back(node);
      ++node_index;
    }
  }
  way_nodes->nodes = nodes;
  return true;
}

bool TrajectoryProcessor::GetRightmostNaviFile(const FileInfo& file_info,
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

void TrajectoryProcessor::GetProcessedFilesInfo(
    const std::map<std::uint16_t, FileInfo>** processed_files_info) {
  *processed_files_info = &processed_file_info_s_;
}

bool TrajectoryProcessor::ResponseToFrontend(
    const NaviResponse& navi_response) {
  std::string msg;
  if (!MessageToJsonString(navi_response, &msg).ok()) {
    AERROR << "Failed to convert from NaviResponse to Json";
    return false;
  }

  {
    std::lock_guard<std::mutex> update_frontend_lock(update_frontend_mut_);
    update_frontend_tasks_.emplace_back(std::bind(update_frontend_func_, msg));
  }
  update_frontend_cv_.notify_one();
  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
