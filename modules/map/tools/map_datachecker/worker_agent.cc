/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker_agent.cpp
 * @desc A agent of MapDataChecker, dispatch process and memory.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/
#include "modules/map/tools/map_datachecker/worker_agent.h"
#include <grpc++/grpc++.h>
#include <memory>

namespace adu {
namespace workers {
namespace collection {

MapDataCheckerAgent::MapDataCheckerAgent() {
    _sp_conf = parse_json(FLAGS_conf_json);
    _sp_pose_collection_agent =
        std::make_shared<PoseCollectionAgent>(_sp_conf);
    _sp_channel_checker_agent =
        std::make_shared<ChannelVerifyAgent>(_sp_conf);

    _sp_static_align_agent =
        std::make_shared<STATIC_ALIGN_AGENT_TYPE>(
            _sp_conf, _sp_pose_collection_agent);
    _sp_eight_route_agent =
        std::make_shared<EIGHT_ROUTE_AGENT_TYPE>(
            _sp_conf, _sp_pose_collection_agent);
    _sp_loops_verify_agent =
        std::make_shared<LoopsVerifyAgent>(
            _sp_conf, _sp_pose_collection_agent);
    AINFO << "MapDataCheckerAgent create successfully";
}

void MapDataCheckerAgent::set_worker_cyber_node(
    std::shared_ptr<MapDataCheckerCyberNode> cyber_node) {
    if (!cyber_node) {
        AFATAL << "Error in create MapDataCheckerCyberNode";
        apollo::cyber::WaitForShutdown();
        apollo::cyber::Clear();
    }
    _cyber_node = cyber_node->get_worker_cyber_node();
}

std::shared_ptr<PoseCollectionAgent>
MapDataCheckerAgent::get_sp_pose_collection_agent() {
    return _sp_pose_collection_agent;
}

grpc::Status
MapDataCheckerAgent::ChannelVerify(
    grpc::ServerContext *context,
    CHANNEL_VERIFY_REQUEST_TYPE *request,
    CHANNEL_VERIFY_RESPONSE_TYPE *response) {
    return _sp_channel_checker_agent->process_grpc_request(
        context, request, response);
}

grpc::Status
MapDataCheckerAgent::StaticAlign(
    grpc::ServerContext *context,
    STATIC_REQUEST_TYPE *request,
    STATIC_RESPONSE_TYPE *response) {
    return _sp_static_align_agent->process_grpc_request(
        context, request, response);
}

grpc::Status
MapDataCheckerAgent::EightRoute(
    grpc::ServerContext *context,
    EIGHTROUTE_REQUEST_TYPE *request,
    EIGHTROUTE_RESPONSE_TYPE *response) {
    return _sp_eight_route_agent->process_grpc_request(
        context, request, response);
}

grpc::Status
MapDataCheckerAgent::LoopsVerify(
    grpc::ServerContext *context,
    LOOPS_VERIFY_REQUEST_TYPE *request,
    LOOPS_VERIFY_RESPONSE_TYPE *response) {
    return _sp_loops_verify_agent->process_grpc_request(
        context, request, response);
}

}  // namespace collection
}  // namespace workers
}  // namespace adu
