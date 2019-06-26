/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/map/tools/map_datachecker/server/channel_verify.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem.hpp>
#include <unordered_map>

#include "cyber/cyber.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_viewer.h"

namespace apollo {
namespace hdmap {

ChannelVerify::ChannelVerify(std::shared_ptr<JSonConf> sp_conf)
    : sp_conf_(sp_conf) {
  reset();
}

void ChannelVerify::reset() {
  return_state_ = ErrorCode::SUCCESS;
  checked_records_.clear();
  sp_vec_check_result_ =
      std::make_shared<std::vector<OneRecordChannelCheckResult>>();
}

ErrorCode ChannelVerify::check(
    const std::string& record_dir_or_record_full_path) {
  std::vector<std::string> records_path;
  records_path = get_records_path(record_dir_or_record_full_path);
  if (records_path.size() == 0) {
    AINFO << "have no data file to check";
    return_state_ = ErrorCode::ERROR_VERIFY_NO_RECORDERS;
    return return_state_;
  }
  incremental_check(records_path);
  return_state_ = ErrorCode::SUCCESS;
  return return_state_;
}

std::shared_ptr<std::vector<OneRecordChannelCheckResult>>
ChannelVerify::get_check_result() const {
  return sp_vec_check_result_;
}

int ChannelVerify::incremental_check(
    const std::vector<std::string>& records_path) {

  std::vector<std::string> not_check_records_path;
  AINFO << "all records path:";
  for (size_t i = 0; i < records_path.size(); ++i) {
    AINFO << "[" << i << "]: " << records_path[i];
    if (is_record_file(records_path[i]) &&
        !is_record_checked(records_path[i])) {
      not_check_records_path.push_back(records_path[i]);
    }
  }

  AINFO << "not_check_records_path:";
  for (size_t i = 0; i < not_check_records_path.size(); ++i) {
    AINFO << "[" << i << "]: " << not_check_records_path[i];
    OneRecordChannelCheckResult check_result =
        check_record_channels(not_check_records_path[i]);
    if (check_result.record_path.empty()) {
      continue;
    }
    sp_vec_check_result_->push_back(check_result);
  }

  return 0;
}

bool ChannelVerify::is_record_file(const std::string& record_path) const {
  if (!boost::filesystem::exists(record_path)) {
    AINFO << "path [" << record_path << "] does not exist";
    return false;
  }
  if (!boost::filesystem::is_regular_file(record_path)) {
    AINFO << "path [" << record_path << "] is not a regular file";
    return false;
  }
  // To avoid disk overhead caused by opening files twice, the real
  // file checking is placed in the function [ChannelVerify::get_record_info]
  return true;
}

std::vector<std::string> ChannelVerify::get_records_path(
    const std::string& record_dir_or_record_full_path) const {
  // record_dir_or_record_full_path is record fullpath or
  // directory which contains some records
  std::vector<std::string> records_path;
  // 1. check record_dir_or_record_full_path is valid or not
  boost::filesystem::path path(record_dir_or_record_full_path);
  if (!boost::filesystem::exists(path)) {
    AINFO << "record path [" << record_dir_or_record_full_path
          << "] does not exist";
    return records_path;
  }

  if (is_record_file(record_dir_or_record_full_path)) {
    records_path.push_back(record_dir_or_record_full_path);
  } else if (boost::filesystem::is_directory(path)) {
    using dit_t = boost::filesystem::directory_iterator;
    dit_t end;
    for (dit_t it(record_dir_or_record_full_path); it != end; ++it) {
      if (is_record_file(it->path().string())) {
        records_path.push_back(it->path().string());
      }
    }
  }
  return records_path;
}

bool ChannelVerify::is_record_checked(const std::string& record_path) {
  if (checked_records_.find(record_path) == checked_records_.end()) {
    checked_records_.insert(record_path);
    return false;
  } else {
    return true;
  }
}

std::shared_ptr<CyberRecordInfo> ChannelVerify::get_record_info(
    const std::string& record_path) const {
  if (!is_record_file(record_path)) {
    AINFO << "get_record_info failed.[" << record_path
          << "] is not record file";
    return nullptr;
  }
  std::shared_ptr<CyberRecordInfo> sp_record_info(new CyberRecordInfo);
  std::shared_ptr<apollo::cyber::record::RecordReader> sp_reader =
      std::make_shared<apollo::cyber::record::RecordReader>(record_path);
  if (sp_reader == nullptr || !sp_reader->IsValid()) {
    AINFO << "open record [" << record_path << "] failed";
    return nullptr;
  }
  std::shared_ptr<apollo::cyber::record::RecordViewer> sp_viewer(
      new apollo::cyber::record::RecordViewer(sp_reader));
  sp_record_info->path = record_path;
  sp_record_info->start_time = sp_viewer->begin_time();
  sp_record_info->end_time = sp_viewer->end_time();
  sp_record_info->duration =
      static_cast<double>((sp_viewer->end_time() - sp_viewer->begin_time())) /
      1e9;

  std::set<std::string> channel_list = sp_reader->GetChannelList();
  for (auto it = channel_list.begin(); it != channel_list.end(); ++it) {
    const std::string& channel_name = *it;
    CyberRecordChannel channel;
    channel.channel_name = channel_name;
    channel.msgnum = sp_reader->GetMessageNumber(channel_name);
    channel.msg_type = sp_reader->GetMessageType(channel_name);
    sp_record_info->channels.push_back(channel);
  }
  return sp_record_info;
}

OneRecordChannelCheckResult ChannelVerify::check_record_channels(
    const std::string& record_path) {
  OneRecordChannelCheckResult check_result;
  std::shared_ptr<CyberRecordInfo> sp_record_info =
      get_record_info(record_path);
  if (sp_record_info == nullptr) {
    return check_result;
  }
  std::vector<CyberRecordChannel>& channels = sp_record_info->channels;
  std::vector<std::pair<std::string, double>>& topic_list =
      sp_conf_->topic_list;
  check_result.record_path = record_path;
  check_result.start_time = sp_record_info->start_time;
  for (size_t i = 0; i < topic_list.size(); ++i) {
    std::string& channel_in_list = topic_list[i].first;
    double channel_expected_rate = topic_list[i].second;
    bool channel_in_list_found = false;
    size_t j = 0;
    for (j = 0; j < channels.size(); ++j) {
      std::string& channel_in_record = channels[j].channel_name;
      if (channel_in_record == channel_in_list) {
        channel_in_list_found = true;
        break;
      }
    }
    if (!channel_in_list_found) {  // topic
      AINFO << record_path << " lacks [" << channel_in_list << "]";
      check_result.lack_channels.push_back(channel_in_list);
    } else {  // rate
      double actual_rate =
          static_cast<double>((channels[j].msgnum)) / sp_record_info->duration;
      if (actual_rate < 1e-8) {
        actual_rate = 0.0;
        AINFO << "msgnum:" << channels[j].msgnum
              << ",duration:" << sp_record_info->duration;
      }
      AINFO << record_path << " [" << channel_in_list
            << "] expected rate: " << channel_expected_rate
            << ", actual rate: " << actual_rate;
      if (actual_rate <
          channel_expected_rate * sp_conf_->topic_rate_tolerance) {
        check_result.inadequate_rate[channel_in_list] =
            std::make_pair(channel_expected_rate, actual_rate);
      }
    }
  }
  return check_result;
}

ErrorCode ChannelVerify::get_return_state() const { return return_state_; }

}  // namespace hdmap
}  // namespace apollo
