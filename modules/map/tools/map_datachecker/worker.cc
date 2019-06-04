/******************************************************************************
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file worker.cpp
 * @desc A worker for demo show, create grpc channel between app.
 * @author Tong Wu<wutong14@baidu.com>
 *****************************************************************************/
#include "modules/map/tools/map_datachecker/worker.h"
#include "modules/map/tools/map_datachecker/worker_cyber_node.h"
#include "modules/map/tools/map_datachecker/worker_agent.h"
#include "modules/map/tools/map_datachecker/worker_gflags.h"
#include <grpc++/grpc++.h>
#include <memory>
#include "cyber/cyber.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;

namespace adu {
namespace workers {
namespace collection {

bool Mapdatachecker::Init() {
    _grpc_address =
        FLAGS_map_datachecker_host + ":" + FLAGS_map_datachecker_port;
    return true;
}

bool Mapdatachecker::Start() {
    AINFO << "Mapdatachecker::Start";
    Init();

    AINFO << "creating agent";
    std::shared_ptr<MapDataCheckerAgent>
        agent = std::make_shared<MapDataCheckerAgent>();

    AINFO << "creating node";
    bool cyber_node_inited = false;
    std::shared_ptr<MapDataCheckerCyberNode> cyber_node =
            std::make_shared<MapDataCheckerCyberNode>(
                agent, &cyber_node_inited);
    if (!cyber_node_inited) {
        AFATAL << "Error in create MapDataCheckerCyberNode";
        apollo::cyber::WaitForShutdown();
        apollo::cyber::Clear();
        return false;
    }
    agent->set_worker_cyber_node(cyber_node);

    AINFO << "register service";
    ServerBuilder builder;
    builder.AddListeningPort(_grpc_address, grpc::InsecureServerCredentials());
    builder.RegisterService(agent.get());
    std::unique_ptr<Server> server(builder.BuildAndStart());
    // server->Wait();
    AINFO << "Server listening on " << _grpc_address;
    apollo::cyber::WaitForShutdown();
    apollo::cyber::Clear();
    return true;
}

bool Mapdatachecker::Stop() {
    return true;
}

void Mapdatachecker::Report() {
}

}  // namespace collection
}  // namespace workers
}  // namespace adu
