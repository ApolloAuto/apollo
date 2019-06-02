/******************************************************************************
 * Created on Wed Jan 09 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_CHANNEL_CHEKCER_AGENT_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_CHANNEL_CHEKCER_AGENT_H

#include <grpc++/grpc++.h>
#include <memory>
#include <mutex>
#include "channel_verify.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
// #include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"


namespace adu {
namespace workers {
namespace collection {

enum class ChannelVerifyAgentState {
    IDLE,
    RUNNING
};

typedef const adu::workers::collection::ChannelVerifyRequest CHANNEL_VERIFY_REQUEST_TYPE;
typedef adu::workers::collection::ChannelVerifyResponse CHANNEL_VERIFY_RESPONSE_TYPE;

class ChannelVerifyAgent {
public:
    ChannelVerifyAgent(std::shared_ptr<JSonConf> sp_conf);    
    grpc::Status process_grpc_request(grpc::ServerContext *context, CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response);
    
private:

    void start_check(CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response);
    void async_check(const std::string& records_path);
    void do_check(const std::string& records_path);
    void check_result(CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response);
    void stop_check(CHANNEL_VERIFY_REQUEST_TYPE *request, CHANNEL_VERIFY_RESPONSE_TYPE *response);
    void reset();
    void set_state(ChannelVerifyAgentState state);
    ChannelVerifyAgentState get_state();
    int add_topic_lack(adu::workers::collection::VerifyResult *result, std::string& record_path, std::vector<std::string>& lack_channels);
    int add_inadequate_rate(adu::workers::collection::VerifyResult *result, std::string& record_path, \
        std::map<std::string, std::pair<double, double>>& inadequate_rate);
    ::adu::workers::collection::FrameRate* find_rates(adu::workers::collection::VerifyResult *result, const std::string& channel);

private: //debug only
    void print_sp_check_result();

private:
    ChannelVerifyAgentState _state;
    std::mutex _stop_mutex;
    bool _need_stop, _stopped;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<ChannelVerify> _sp_channel_checker = nullptr;
    CheckResult _sp_check_result = nullptr;
    std::thread::id _check_thread_id;
};

} // collection
} // workers
} // adu


#endif // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_CHANNEL_CHEKCER_AGENT_H
