/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_CHANNEL_CHECKER_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_CHANNEL_CHECKER_H
#include <vector>
#include <string>
#include <map>
#include <set>
#include <memory>
#include <utility>
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

namespace apollo {
namespace hdmap {

struct CyberRecordChannel {
    std::string channel_name;
    uint64_t msgnum;
    std::string msg_type;
};

struct CyberRecordInfo {
    std::string path;
    // std::string version;
    double duration;  // int sec
    uint64_t start_time;
    uint64_t end_time;
    // int msgnum;
    // int chunknum;
    // uint64_t size; //bytes
    std::vector<CyberRecordChannel> channels;
};

struct OneRecordChannelCheckResult {
    std::string record_path;
    uint64_t start_time;
    std::vector<std::string> lack_channels;
    // inadequate_rate: channel_name <------> (expected_rate, actual_rate)
    std::map<std::string, std::pair<double, double>> inadequate_rate;
};

typedef std::shared_ptr<std::vector<OneRecordChannelCheckResult>> CheckResult;
typedef std::vector<OneRecordChannelCheckResult>::iterator CheckResultIterator;

class ChannelVerify {
 public:
    explicit ChannelVerify(std::shared_ptr<JSonConf> sp_conf);
    ErrorCode check(std::string record_dir_or_record_full_path);
    std::shared_ptr<std::vector<OneRecordChannelCheckResult>>
        get_check_result();
    ErrorCode get_return_state();
 private:
    bool is_record_file(std::string path);
    std::shared_ptr<CyberRecordInfo> get_record_info(std::string record_path);
    int incremental_check(std::vector<std::string> records_path);
    std::vector<std::string> get_records_path(
        std::string record_dir_or_record_full_path);
    bool is_record_checked(std::string record_path);
    OneRecordChannelCheckResult check_record_channels(std::string record_path);
    void reset();
 private:
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    CheckResult _sp_vec_check_result = nullptr;
    ErrorCode _return_state;
    std::set<std::string> _checked_records;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_CHANNEL_CHECKER_H
