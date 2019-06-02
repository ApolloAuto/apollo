/******************************************************************************
 * Created on Wed Jan 09 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#include <thread>
#include <chrono>
#include "channel_verify_agent.h"

namespace adu {
namespace workers {
namespace collection {

ChannelVerifyAgent::ChannelVerifyAgent(std::shared_ptr<JSonConf> sp_conf) {
    _sp_conf = sp_conf;
    _sp_channel_checker = nullptr;
    _sp_check_result = nullptr;
    //reset();
}

void ChannelVerifyAgent::reset() {
    std::lock_guard<std::mutex> guard(_stop_mutex);
    _need_stop = false, _stopped = false;
    _sp_channel_checker = std::make_shared<ChannelVerify>(_sp_conf);
    _sp_check_result = nullptr;
    set_state(ChannelVerifyAgentState::IDLE);
}

grpc::Status ChannelVerifyAgent::process_grpc_request(grpc::ServerContext *context, CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response) {
    AINFO << "ChannelVerifyAgent Request: " << request->DebugString();
    switch (request->cmd())
    {
    case CmdType::START:
        AINFO << "ChannelVerifyAgent start";
        start_check(request, response);
        break;
    case CmdType::CHECK:
        AINFO << "ChannelVerifyAgent check";
        check_result(request, response);
        break;
    case CmdType::STOP:
        AINFO << "ChannelVerifyAgent stop";
        stop_check(request, response);
        break;
    default:
        response->set_code(ErrorCode::ERROR_REQUEST);
        AERROR << "command error";
    }
    AINFO << "ChannelVerifyAgent progress: " << response->DebugString();
    return grpc::Status::OK;
}

void ChannelVerifyAgent::start_check(CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response) {
    if ( get_state() == ChannelVerifyAgentState::RUNNING ) {
        AINFO << "ChannelVerify is RUNNING, do not need start again";
        response->set_code(ErrorCode::ERROR_REPEATED_START);
        return;
    }
    reset();
    std::string records_path = request->path();
    async_check(records_path);
    response->set_code(ErrorCode::SUCCESS);
}

void ChannelVerifyAgent::async_check(const std::string& records_path) {
    set_state(ChannelVerifyAgentState::RUNNING);
    std::thread doctor_strange([=](){
        _check_thread_id = std::this_thread::get_id();
        int wait_sec = _sp_conf->channel_check_trigger_gap;
        while(true) {
            {
                std::lock_guard<std::mutex> guard(_stop_mutex);
                if (_need_stop) {
                     break;
                }
                do_check(records_path);
                AINFO << "thread check done";
            }
            std::this_thread::sleep_for( std::chrono::seconds(wait_sec) );
            if ( std::this_thread::get_id() != _check_thread_id ) {
                break;
            }
        }
        _stopped = true;
    });
    doctor_strange.detach();
    AINFO << "ChannelVerifyAgent::async_check exit";
}

void ChannelVerifyAgent::do_check(const std::string& records_path) {
    if ( _sp_channel_checker == nullptr) {
        _sp_channel_checker = std::make_shared<ChannelVerify>(_sp_conf);
    }
    set_state(ChannelVerifyAgentState::RUNNING);
    _sp_channel_checker->check(records_path);
    _sp_check_result = _sp_channel_checker->get_check_result();
}

int ChannelVerifyAgent::add_topic_lack(adu::workers::collection::VerifyResult *result, std::string& record_path, std::vector<std::string>& lack_channels) {
    adu::workers::collection::TopicResult *topics = result->mutable_topics();
    for(size_t i = 0; i < lack_channels.size(); i++) {
        topics->add_topic_lack(lack_channels[i]);
        AINFO << record_path << " lack topic: " << lack_channels[i];
    }
    return int(lack_channels.size());
}

::adu::workers::collection::FrameRate* ChannelVerifyAgent::find_rates(adu::workers::collection::VerifyResult *result, const std::string& channel) {
    int rates_size = result->rates_size();
    for ( int i = 0; i < rates_size; i++ ) {
        const ::adu::workers::collection::FrameRate& rates = result->rates(i);
        if ( rates.topic() == channel ) {
            return result->mutable_rates(i);
        }
    }
    return NULL;
}

int ChannelVerifyAgent::add_inadequate_rate(adu::workers::collection::VerifyResult *result, std::string& record_path, \
    std::map<std::string, std::pair<double, double>>& inadequate_rate) {

    for(std::map<std::string, std::pair<double, double>>::iterator it = inadequate_rate.begin(); it != inadequate_rate.end(); ++it) {
        const std::string& channel = it->first;
        double expected_rate = it->second.first;
        double current_rate = it->second.second;
        ::adu::workers::collection::FrameRate* rate = find_rates(result, channel);
        if (rate == NULL) {
            rate = result->add_rates();
            rate->add_bad_record_name(record_path);
            rate->set_topic(channel);
            rate->set_expected_rate(expected_rate);
            rate->set_current_rate(current_rate);
        } else {
            rate->set_current_rate(current_rate);
            rate->add_bad_record_name(record_path);
        }
    }
    return result->rates_size();
}

void ChannelVerifyAgent::print_sp_check_result() {
/*    fprintf(stderr, "print_sp_check_result\n");
    for( CheckResultIterator it = _sp_check_result->begin(); it != _sp_check_result->end(); it++ ) {
        fprintf(stderr, "%s\n", it->record_path.c_str());
        std::vector<std::string>& lack_channels = it->lack_channels;
        for(int i = 0; i < lack_channels.size(); i++) {
            fprintf(stderr, "-->[lack]%s\n", lack_channels[i].c_str());
        }
        std::map<std::string, std::pair<double, double>> & inadequate_rate = it->inadequate_rate;
        for( std::map<std::string, std::pair<double, double>>::iterator it1 = inadequate_rate.begin(); it1 != inadequate_rate.end(); it1++ ) {
            fprintf(stderr, "-->[inadequate_rate]%s expected_rate:%lf actual_rate:%lf\n", it1->first.c_str(), it1->second.first, it1->second.second);
        }
    }*/
}


void ChannelVerifyAgent::check_result(CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response) {
    if ( get_state() == ChannelVerifyAgentState::IDLE ) {
        AINFO << "ChannelVerify is not RUNNING, it should start first";
        response->set_code(ErrorCode::ERROR_CHECK_BEFORE_START);
        return;
    }

    if ( _sp_channel_checker == nullptr || _sp_check_result == nullptr ) {
        AINFO << "check result is null. check later";
        response->set_code(ErrorCode::SUCCESS);
        return;
    }

    if ( _sp_channel_checker->get_return_state() != ErrorCode::SUCCESS ) {
        response->set_code(_sp_channel_checker->get_return_state());
        return;
    }

    print_sp_check_result(); // debug purpose
    
    response->set_code(ErrorCode::SUCCESS);
    adu::workers::collection::VerifyResult *result = response->mutable_result();
    for( CheckResultIterator it = _sp_check_result->begin(); it != _sp_check_result->end(); it++ ) {
        int res = 0;
        // write rate
        res = add_inadequate_rate(result, it->record_path, it->inadequate_rate);
        if ( res > 0 ) {
            response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_RATES_ABNORMAL);
        }
        // write topic lack
        res = add_topic_lack(result, it->record_path, it->lack_channels);
        if ( res > 0 ) {
            response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_TOPIC_LACK);
        }
    }
}

void ChannelVerifyAgent::stop_check(CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response) {
    std::lock_guard<std::mutex> guard(_stop_mutex);
    _need_stop = true;
    response->set_code(ErrorCode::SUCCESS);
    set_state(ChannelVerifyAgentState::IDLE);
    adu::workers::collection::VerifyResult *result = response->mutable_result();
    if (_sp_check_result == nullptr) {
        return;
    }
    for( CheckResultIterator it = _sp_check_result->begin(); it != _sp_check_result->end(); it++ ) {
        int res = 0;
        // write rate
        res = add_inadequate_rate(result, it->record_path, it->inadequate_rate);
        if ( res > 0 ) {
            response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_RATES_ABNORMAL);
        }
        // write topic lack
        res = add_topic_lack(result, it->record_path, it->lack_channels);
        if ( res > 0 ) {
            response->set_code(ErrorCode::ERROR_CHANNEL_VERIFY_TOPIC_LACK);
        }
    }   
}

void ChannelVerifyAgent::set_state(ChannelVerifyAgentState state) {
    _state = state;
}
ChannelVerifyAgentState ChannelVerifyAgent::get_state() {
    return _state;
}

} // collection
} // workers
} // adu
