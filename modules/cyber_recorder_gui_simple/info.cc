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

#include "modules/cyber_recorder_gui_simple/info.h"

#include "cyber/record/record_message.h"

namespace apollo {
namespace goodman {
namespace record {

// using apollo::cyber::proto::ChannelCache;
// using apollo::cyber::proto::Header;
// using apollo::cyber::proto::Index;
// using apollo::cyber::proto::SectionType;
using namespace apollo::cyber::proto;
using namespace apollo::cyber::record;
using apollo::cyber::record::kGB;
using apollo::cyber::record::kKB;
using apollo::cyber::record::kMB;
// using apollo::cyber::record::RecordFileReader;
Info::Info() {}

Info::~Info() {}

bool Info::Display(const std::string& file, std::string* info_text) {
    std::ostringstream channel_info_oss;

    apollo::cyber::record::RecordFileReader file_reader;

    if (!file_reader.Open(file)) {
        AERROR << "open record file error. file: " << file;
        return false;
    }
    Header hdr = file_reader.GetHeader();

    channel_info_oss << setiosflags(std::ios::left);
    channel_info_oss << setiosflags(std::ios::fixed);

    int w = 16;
    // file name
    channel_info_oss << std::setw(w) << "record_file: " << file << std::endl;

    // version
    channel_info_oss << std::setw(w) << "version: " << hdr.major_version() << "." << hdr.minor_version() << std::endl;

    // time and duration
    auto begin_time_s = static_cast<double>(hdr.begin_time()) / 1e9;
    auto end_time_s = static_cast<double>(hdr.end_time()) / 1e9;
    auto duration_s = end_time_s - begin_time_s;
    auto begin_time_str = UnixSecondsToString(static_cast<int>(begin_time_s));
    auto end_time_str = UnixSecondsToString(static_cast<int>(end_time_s));
    channel_info_oss << std::setw(w) << "duration: " << duration_s << " Seconds" << std::endl;
    channel_info_oss << std::setw(w) << "begin_time: " << begin_time_str << std::endl;
    channel_info_oss << std::setw(w) << "end_time: " << end_time_str << std::endl;

    // size
    channel_info_oss << std::setw(w) << "size: " << hdr.size() << " Bytes";
    if (hdr.size() >= kGB) {
        channel_info_oss << " (" << static_cast<float>(hdr.size()) / kGB << " GB)";
    } else if (hdr.size() >= kMB) {
        channel_info_oss << " (" << static_cast<float>(hdr.size()) / kMB << " MB)";
    } else if (hdr.size() >= kKB) {
        channel_info_oss << " (" << static_cast<float>(hdr.size()) / kKB << " KB)";
    }
    channel_info_oss << std::endl;

    // is_complete
    channel_info_oss << std::setw(w) << "is_complete:";
    if (hdr.is_complete()) {
        channel_info_oss << "true";
    } else {
        channel_info_oss << "false";
    }
    channel_info_oss << std::endl;

    // message_number
    channel_info_oss << std::setw(w) << "message_number: " << hdr.message_number() << std::endl;

    // channel_number
    channel_info_oss << std::setw(w) << "channel_number: " << hdr.channel_number() << std::endl;

    // read index section
    if (!file_reader.ReadIndex()) {
        AERROR << "read index section of the file fail. file: " << file;
        return false;
    }

    // channel info
    channel_info_oss << std::setw(w) << "channel_info: " << std::endl;

    Index idx = file_reader.GetIndex();
    for (int i = 0; i < idx.indexes_size(); ++i) {
        ChannelCache* cache = idx.mutable_indexes(i)->mutable_channel_cache();
        if (idx.mutable_indexes(i)->type() == SectionType::SECTION_CHANNEL) {
            channel_info_oss << std::setw(w) << "";
            channel_info_oss << resetiosflags(std::ios::right);
            channel_info_oss << std::setw(50) << cache->name();
            channel_info_oss << setiosflags(std::ios::right);
            channel_info_oss << std::setw(8) << cache->message_number();
            channel_info_oss << std::setw(0) << " messages: ";
            channel_info_oss << cache->message_type();
            channel_info_oss << std::endl;
        }
    }
    *info_text = channel_info_oss.str();

    file_reader.Close();
    return true;
}

}  // namespace record
}  // namespace goodman
}  // namespace apollo
