/******************************************************************************
 * Created on Wed Jan 09 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "channel_verify.h"
#include "cyber/cyber.h"
#include "cyber/record/record_viewer.h"
#include "cyber/proto/record.pb.h"

namespace adu {
namespace workers {
namespace collection {

ChannelVerify::ChannelVerify(std::shared_ptr<JSonConf> sp_conf) {
    _sp_conf = sp_conf;
    reset();
}

void ChannelVerify::reset() {
    _return_state = ErrorCode::SUCCESS;
    _checked_records.clear();
    _sp_vec_check_result = std::make_shared<std::vector<OneRecordChannelCheckResult>>();
}

ErrorCode ChannelVerify::check(std::string record_dir_or_record_full_path) {
    std::vector<std::string> records_path = get_records_path(record_dir_or_record_full_path);
    if (records_path.size() == 0)
    {
        AINFO << "have no data file to check";
        _return_state = ErrorCode::ERROR_VERIFY_NO_RECORDERS;
        return _return_state; // ERROR_VERIFY_NO_RECORDERS
    }
    incremental_check(records_path);
    _return_state = ErrorCode::SUCCESS;
    return _return_state; 
}

std::shared_ptr<std::vector<OneRecordChannelCheckResult>> ChannelVerify::get_check_result() {
    return _sp_vec_check_result;
}

int ChannelVerify::incremental_check(std::vector<std::string> records_path) {
    // 1.
    std::vector<std::string> not_check_records_path;
    AINFO << "all records path:";
    for(size_t i = 0; i < records_path.size(); i++) {
        AINFO << "[" << i << "]: " << records_path[i];
        if( ! is_record_checked(records_path[i]) ) {
            not_check_records_path.push_back(records_path[i]);
        }
    } 

    // 2.
    AINFO << "not_check_records_path:";
    for(size_t i = 0; i < not_check_records_path.size(); i++) {
        AINFO << "[" << i << "]: " << not_check_records_path[i];
        //std::shared_ptr<CyberRecordInfo> sp_record_info = get_record_info(not_check_records_path[i]);
        //if (sp_record_info == nullptr) {
        //    AINFO << "get_record_info failed. record:" << not_check_records_path[i];
        //    continue;
        //}
        OneRecordChannelCheckResult check_result = check_record_channels(not_check_records_path[i]);
        _sp_vec_check_result->push_back(check_result);
    }

    return 0;
}

bool ChannelVerify::is_record_file(std::string record_path) {
    if (!boost::filesystem::exists(record_path)) {
        AINFO << "path [" << record_path << "] is not exist";
        return false;
    }
    if (!boost::filesystem::is_regular_file(record_path)) {
        AINFO << "path [" << record_path << "] is not a regular file";
        return false;
    }
    if (boost::filesystem::extension(record_path) != ".record") {
        AINFO << "path [" << record_path << "] extension is not record";
        return false;
    }
    // TODO
    // To avoid disk overhead caused by opening files twice, the real file checking is placed in the function [ChannelVerify::get_record_info]
    return true;  
}


std::vector<std::string> ChannelVerify::get_records_path(std::string record_dir_or_record_full_path) {
    // record_dir_or_record_full_path is record fullpath or directory which contains some records
    std::vector<std::string> records_path;
    // 1. check record_dir_or_record_full_path is valid or not
    boost::filesystem::path path(record_dir_or_record_full_path);
    if (!boost::filesystem::exists(path)) {
        AINFO << "record path [" << record_dir_or_record_full_path << "] is not exist";
        return records_path;
    }

    if (is_record_file(record_dir_or_record_full_path)) {
        records_path.push_back(record_dir_or_record_full_path);
    }
    else if (boost::filesystem::is_directory(path)) {
        boost::filesystem::directory_iterator end;
        for (boost::filesystem::directory_iterator it(record_dir_or_record_full_path); it != end; it++) {
            if (is_record_file(it->path().string())) {
                records_path.push_back(it->path().string());
            }
        }
    }
    return records_path;
}

bool ChannelVerify::is_record_checked(std::string record_path) {
    if (_checked_records.find(record_path) == _checked_records.end()) {
        _checked_records.insert(record_path);
        return false;
    } else {
        return true;
    }
}

std::shared_ptr<CyberRecordInfo> ChannelVerify::get_record_info(std::string record_path) {
    if (!is_record_file(record_path) ) {
        AINFO << "get_record_info failed.[" << record_path << "] is not record file";
        return nullptr;
    }
    std::shared_ptr<CyberRecordInfo> sp_record_info(new CyberRecordInfo);
    // std::shared_ptr<cybertron::DataFile> data_file = std::make_shared<cybertron::DataFile>();
    // if ( cybertron::FAIL == data_file->open(record_path, cybertron::FileMode::Read) ) {
    //     AINFO << "open record [" << record_path << "] failed";
    //     return nullptr;
    // }
    std::shared_ptr<apollo::cyber::record::RecordReader> sp_reader(new apollo::cyber::record::RecordReader(record_path));
    // apollo::cyber::record::RecordReader reader(record_path);
    if (sp_reader == nullptr or !sp_reader->IsValid()) {
        AINFO << "open record [" << record_path << "] failed";
        return nullptr;
    }
    // apollo::cyber::record::RecordViewer viewer(sp_reader);
    std::shared_ptr<apollo::cyber::record::RecordViewer> sp_viewer(new apollo::cyber::record::RecordViewer(sp_reader));

    sp_record_info->path = record_path; //data_file->get_file_name();
    // sp_record_info->version = data_file->get_version();
    sp_record_info->start_time = sp_viewer->begin_time();
    sp_record_info->end_time = sp_viewer->end_time();
    sp_record_info->duration = (double)( sp_viewer->end_time() - sp_viewer->begin_time() ) / 1e9;

    // sp_record_info->msgnum = data_file->get_msg_num();
    //sp_record_info->chunknum = 
    // sp_record_info->size = data_file->get_file_size();
    // std::set<std::string> channels = data_file->GetChannelList();
    using ChannelInfoMap = std::unordered_map<std::string, apollo::cyber::proto::ChannelCache>;
    ChannelInfoMap channel_info = sp_reader->channel_info();
    for(ChannelInfoMap::iterator it = channel_info.begin(); it != channel_info.end(); it++) {
        CyberRecordChannel channel;
        channel.channel_name = it->second.name();
        channel.msgnum = (uint64_t)it->second.message_number();
        channel.msg_type = it->second.message_type();
        sp_record_info->channels.push_back(channel);
    }
    return sp_record_info;
}

OneRecordChannelCheckResult ChannelVerify::check_record_channels(std::string record_path) {
    std::shared_ptr<CyberRecordInfo> sp_record_info = get_record_info(record_path);
    std::vector<CyberRecordChannel>& channels = sp_record_info->channels;
    std::vector<std::pair<std::string, double>> &topic_list = _sp_conf->topic_list;

    OneRecordChannelCheckResult check_result;
    check_result.record_path = record_path;
    check_result.start_time = sp_record_info->start_time;
    for(size_t i = 0; i < topic_list.size(); i++) {
        std::string& channel_in_list = topic_list[i].first;
        double channel_expected_rate = topic_list[i].second;
        bool channel_in_list_found = false;
        size_t j = 0;
        for(j = 0; j < channels.size(); j++) {
            std::string& channel_in_record = channels[j].channel_name;
            if( channel_in_record == channel_in_list ) {
                channel_in_list_found = true;
                break;
            }
        }
        if ( !channel_in_list_found ) { // topic
            AINFO << record_path << " lacks [" <<  channel_in_list << "]";
            check_result.lack_channels.push_back(channel_in_list);
        } else {  // rate
            double actual_rate = double(channels[j].msgnum) / sp_record_info->duration;
            if ( actual_rate < 1e-8 ) {
                actual_rate = 0.0;
                AINFO << "msgnum:" << channels[j].msgnum << ",duration:" << sp_record_info->duration;
            }
            AINFO << record_path << " [" << channel_in_list  << "] expected rate: " << channel_expected_rate << ", actual rate: " << actual_rate;
            if ( actual_rate < channel_expected_rate * _sp_conf->topic_rate_tolerance ) {
                check_result.inadequate_rate[channel_in_list] = std::make_pair(channel_expected_rate, actual_rate);
            }
        }
    }
    return check_result; 
}

ErrorCode ChannelVerify::get_return_state() {
    return _return_state;
}


} // collection
} // workers
} // adu
