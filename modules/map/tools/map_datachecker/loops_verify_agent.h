/******************************************************************************
 * Created on Thu Jan 17 2019
 *
 * Copyright (c) 2019 Baidu.com, Inc. All Rights Reserved
 *
 * @file: filename
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/
#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_LOOPS_VERIFY_AGENT_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_LOOPS_VERIFY_AGENT_H
#include <grpc++/grpc++.h>
#include <memory>
#include <utility>
#include <vector>
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/laps_checker.h"
#include "modules/map/tools/map_datachecker/pose_collection_agent.h"

namespace adu {
namespace workers {
namespace collection {

enum class LoopsVerifyAgentState {
    IDLE,
    RUNNING
};
using LOOPS_VERIFY_REQUEST_TYPE
    = const adu::workers::collection::LoopsVerifyRequest;
using LOOPS_VERIFY_RESPONSE_TYPE
    = adu::workers::collection::LoopsVerifyResponse;

class LoopsVerifyAgent {
 public:
    LoopsVerifyAgent(
        std::shared_ptr<JSonConf> sp_conf,
        std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent);
    grpc::Status process_grpc_request(
        grpc::ServerContext *context,
        LOOPS_VERIFY_REQUEST_TYPE *request,
        LOOPS_VERIFY_RESPONSE_TYPE *response);

 private:
    void StartVerify(
        LOOPS_VERIFY_REQUEST_TYPE *request,
        LOOPS_VERIFY_RESPONSE_TYPE *response);
    void CheckVerify(
        LOOPS_VERIFY_REQUEST_TYPE *request,
        LOOPS_VERIFY_RESPONSE_TYPE *response);
    void StopVerify(
        LOOPS_VERIFY_REQUEST_TYPE *request,
        LOOPS_VERIFY_RESPONSE_TYPE *response);
    std::shared_ptr<std::vector<std::pair<double, double>>>
        get_verify_range(LOOPS_VERIFY_REQUEST_TYPE *request);
    size_t get_loops_to_check(LOOPS_VERIFY_REQUEST_TYPE *request);
    int get_poses_to_check(
        std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
        std::vector<FramePose> * poses);
    int do_start_verify(
        std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
        double loops_to_check);
    double get_range_index(
        std::shared_ptr<std::vector<std::pair<double, double>>> sp_range,
        std::vector<bool> * range_index,
        std::shared_ptr<std::vector<FramePose>> sp_vec_poses);
    void set_state(LoopsVerifyAgentState state);
    LoopsVerifyAgentState get_state();

 private:
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<PoseCollectionAgent> _sp_pose_collection_agent = nullptr;
    std::shared_ptr<LapsChecker> _sp_laps_checker = nullptr;
    LoopsVerifyAgentState _state;
};

}  // namespace collection
}  // namespace workers
}  // namespace adu

#endif  // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_LOOPS_VERIFY_AGENT_H
