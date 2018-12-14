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

#include "modules/common/util/util.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"
#include "modules/tools/navi_generator/backend/util/file_operator.h"
#include "modules/tools/navi_generator/backend/util/navigation_expander.h"

namespace apollo {
namespace navi_generator {
namespace util {

namespace {
constexpr double kOverlapLenFromSecondBag = 250.0;
constexpr double kDefaultLaneWidth = 3.75;
}  // namespace

TrajectoryProcessor::TrajectoryProcessor()
    : file_seg_thread_(std::make_unique<std::thread>(
          &TrajectoryProcessor::FileSegmentThread, this)) {}

TrajectoryProcessor::~TrajectoryProcessor() {
  file_seg_finished_ = true;
  if (file_seg_thread_ && file_seg_thread_->joinable()) {
    file_seg_thread_->join();
  }
}

void TrajectoryProcessor::FileSegmentThread() {
  while (!file_seg_finished_) {
    std::unique_lock<std::mutex> lock(file_seg_mut_);
    file_seg_cv_.wait(lock, [this]() {
      return !cur_file_segment_s_.empty() || file_seg_finished_;
    });
    // All segments of a file have been received.
    if (!cur_file_segment_s_.empty()) {
      if (!ExportSegmentsToFile(cur_file_segment_s_[0].cur_file_name)) {
        cur_file_segment_s_.clear();
        // TODO(*): Inform the frontend an error message.
        return;
      }
      BagFileInfo bag_file_info;
      bag_file_info.file_index = cur_file_segment_s_[0].cur_file_index;
      bag_file_info.raw_file_name = cur_file_segment_s_[0].cur_file_name;
      bag_file_info.next_raw_file_name = cur_file_segment_s_[0].next_file_name;
      bag_files_to_process_[bag_file_info.file_index] = bag_file_info;
      cur_file_segment_s_.clear();
    }

    lock.unlock();
    ProcessFiles();
  }
}

bool TrajectoryProcessor::SetCommonBagFileInfo(
    const CommonBagFileInfo& common_file_info) {
  common_file_info_ = common_file_info;
  return true;
}

bool TrajectoryProcessor::ProcessBagFileSegment(
    const FileSegment& file_segment) {
  std::lock_guard<std::mutex> lk(file_seg_mut_);
  cur_file_segment_s_[file_segment.cur_file_seg_index] = file_segment;
  if (cur_file_segment_s_.size() < file_segment.cur_file_seg_count) {
    return true;
  }
  file_seg_cv_.notify_all();
  return true;
}

bool TrajectoryProcessor::SaveFilesToDatabase() {
  // TODO(*): Save all data to database and inform the frontend the result.
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

void TrajectoryProcessor::ProcessFiles() {
  BagFileInfo bag_file_info;
  while (bag_files_to_process_.size() > 0) {
    auto item = bag_files_to_process_.begin();
    bag_file_info = item->second;
    std::string next_file_to_process;
    if (bag_file_info.file_index + 1 == common_file_info_.total_file_num) {
      // the last file to process
      if (!ProcessFile(bag_file_info)) {
        // TODO(*): Inform the frontend an error message.
      } else {
        // TODO(*): Inform the frontend the result.
      }
      bag_files_to_process_.erase(bag_file_info.file_index);
      break;
    }
    if (bag_file_info.file_index == 0) {
      // the first but not last file
      continue;
    }
    // the 2nd~ (total_file_num-1)th file to process
    if (++item != bag_files_to_process_.end()) {
      if (!ProcessFile(bag_file_info)) {
        // TODO(*): Inform the frontend an error message.
        bag_files_to_process_.erase(bag_file_info.file_index);
        break;
      } else {
        // TODO(*): Inform the frontend the result.
        bag_files_to_process_.erase(bag_file_info.file_index);
      }
    }
  }
}

bool TrajectoryProcessor::ProcessFile(const BagFileInfo& bag_file_info) {
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
  // TODO(*): set the speed limits
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

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
