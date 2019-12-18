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
#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/server/common.h"

namespace apollo {
namespace hdmap {

struct CyberRecordChannel {
  std::string channel_name;
  uint64_t msgnum;
  std::string msg_type;
};

struct CyberRecordInfo {
  std::string path;
  double duration;  // the unit is seconds
  uint64_t start_time;
  uint64_t end_time;
  std::vector<CyberRecordChannel> channels;
};

struct OneRecordChannelCheckResult {
  std::string record_path;
  uint64_t start_time;
  std::vector<std::string> lack_channels;
  // inadequate_rate: channel_name <---> (expected_rate, actual_rate)
  std::map<std::string, std::pair<double, double>> inadequate_rate;
};

typedef std::shared_ptr<std::vector<OneRecordChannelCheckResult>> CheckedResult;
typedef std::vector<OneRecordChannelCheckResult>::iterator CheckResultIterator;

class ChannelVerify {
 public:
  explicit ChannelVerify(std::shared_ptr<JSonConf> sp_conf);
  ErrorCode Check(const std::string& record_dir_or_record_full_path);
  std::shared_ptr<std::vector<OneRecordChannelCheckResult>> get_check_result()
      const;
  ErrorCode GetReturnState() const;

 private:
  bool IsRecordFile(const std::string& path) const;
  std::shared_ptr<CyberRecordInfo> GetRecordInfo(
      const std::string& record_path) const;
  int IncrementalCheck(const std::vector<std::string>& records_path);
  std::vector<std::string> GetRecordsPath(
      const std::string& record_dir_or_record_full_path) const;
  bool IsRecordChecked(const std::string& record_path);
  OneRecordChannelCheckResult CheckRecordChannels(
      const std::string& record_path);
  void Reset();

 private:
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  CheckedResult sp_vec_check_result_ = nullptr;
  ErrorCode return_state_;
  std::set<std::string> checked_records_;
};

}  // namespace hdmap
}  // namespace apollo
