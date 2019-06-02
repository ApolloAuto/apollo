/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker_agent.h
 * @desc A agent of MapDataChecker, dispatch process and memory.
 * @author Tong Wu<wutong14@baidu.com>, yuanyijun@baidu.com
 *****************************************************************************/

#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_WORKER_AGENT_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_WORKER_AGENT_H

#include <vector>
#include <utility>
#include <grpc++/grpc++.h>
#include "alignment_agent.hpp"
#include "common.hpp"
#include "worker_cyber_node.h"
#include "pose_collection.h"
#include "channel_verify_agent.h"
#include "loops_verify_agent.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"
namespace adu {
namespace workers {
namespace collection {

class MapDataCheckerAgent final
    : public std::enable_shared_from_this<MapDataCheckerAgent>,
    public CollectionCheckerService::Service
{
  public:
    MapDataCheckerAgent();
    inline std::shared_ptr<MapDataCheckerAgent> get_worker_agent() {
        return shared_from_this();
    }
    void set_worker_cyber_node(std::shared_ptr<MapDataCheckerCyberNode> cyber_node);
    std::shared_ptr<PoseCollectionAgent> get_sp_pose_collection_agent();

    grpc::Status ChannelVerify(grpc::ServerContext*, adu::workers::collection::CHANNEL_VERIFY_REQUEST_TYPE*, adu::workers::collection::CHANNEL_VERIFY_RESPONSE_TYPE*);
    // grpc::Status DynamicAlign(grpc::ServerContext *context, DYMAMIC_REQUEST_TYPE *request, DYNAMIC_RESPONSE_TYPE *response);
    grpc::Status StaticAlign(grpc::ServerContext *context, STATIC_REQUEST_TYPE *request, STATIC_RESPONSE_TYPE *response);
    grpc::Status EightRoute(grpc::ServerContext *context, EIGHTROUTE_REQUEST_TYPE *request, EIGHTROUTE_RESPONSE_TYPE *response);
    grpc::Status LoopsVerify(grpc::ServerContext *context, LOOPS_VERIFY_REQUEST_TYPE *request, LOOPS_VERIFY_RESPONSE_TYPE *response);


private:
    std::shared_ptr<MapDataCheckerCyberNode> _cyber_node = nullptr;

    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<PoseCollectionAgent> _sp_pose_collection_agent = nullptr;
    std::shared_ptr<ChannelVerifyAgent> _sp_channel_checker_agent = nullptr;

    std::shared_ptr<STATIC_ALIGN_AGENT_TYPE> _sp_static_align_agent = nullptr;
    // std::shared_ptr<DYNAMIC_ALIGN_AGENT_TYPE> _sp_dynamic_align_agent = nullptr;
    std::shared_ptr<EIGHT_ROUTE_AGENT_TYPE> _sp_eight_route_agent = nullptr;
    std::shared_ptr<LoopsVerifyAgent> _sp_loops_verify_agent = nullptr;

    
};

} // namespace collection
} // namespace workers
} // namespace adu

#endif // _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_WORKER_AGENT_H
