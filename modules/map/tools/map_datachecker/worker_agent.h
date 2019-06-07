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
#ifndef MODULES_MAP_TOOLS_MAP_DATACHECKER_WORKER_AGENT_H
#define MODULES_MAP_TOOLS_MAP_DATACHECKER_WORKER_AGENT_H
#include <grpc++/grpc++.h>
#include <vector>
#include <utility>
#include <memory>
#include "modules/map/tools/map_datachecker/alignment_agent.hpp"
#include "modules/map/tools/map_datachecker/common.hpp"
#include "modules/map/tools/map_datachecker/worker_cyber_node.h"
#include "modules/map/tools/map_datachecker/pose_collection.h"
#include "modules/map/tools/map_datachecker/channel_verify_agent.h"
#include "modules/map/tools/map_datachecker/loops_verify_agent.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"

namespace apollo {
namespace hdmap {

class MapDataCheckerAgent final:
    public std::enable_shared_from_this<MapDataCheckerAgent>,
    public CollectionCheckerService::Service {
 public:
    MapDataCheckerAgent();
    inline std::shared_ptr<MapDataCheckerAgent> get_worker_agent() {
        return shared_from_this();
    }
    void set_worker_cyber_node(
        std::shared_ptr<MapDataCheckerCyberNode> cyber_node);
    std::shared_ptr<PoseCollectionAgent> get_sp_pose_collection_agent();

    grpc::Status ChannelVerify(
        grpc::ServerContext*,
        apollo::hdmap::CHANNEL_VERIFY_REQUEST_TYPE*,
        apollo::hdmap::CHANNEL_VERIFY_RESPONSE_TYPE*);
    grpc::Status StaticAlign(
        grpc::ServerContext *context,
        STATIC_REQUEST_TYPE *request,
        STATIC_RESPONSE_TYPE *response);
    grpc::Status EightRoute(
        grpc::ServerContext *context,
        EIGHTROUTE_REQUEST_TYPE *request,
        EIGHTROUTE_RESPONSE_TYPE *response);
    grpc::Status LoopsVerify(
        grpc::ServerContext *context,
        LOOPS_VERIFY_REQUEST_TYPE *request,
        LOOPS_VERIFY_RESPONSE_TYPE *response);

 private:
    std::shared_ptr<MapDataCheckerCyberNode> _cyber_node = nullptr;
    std::shared_ptr<JSonConf> _sp_conf = nullptr;
    std::shared_ptr<PoseCollectionAgent> _sp_pose_collection_agent = nullptr;
    std::shared_ptr<ChannelVerifyAgent> _sp_channel_checker_agent = nullptr;
    std::shared_ptr<STATIC_ALIGN_AGENT_TYPE> _sp_static_align_agent = nullptr;
    std::shared_ptr<EIGHT_ROUTE_AGENT_TYPE> _sp_eight_route_agent = nullptr;
    std::shared_ptr<LoopsVerifyAgent> _sp_loops_verify_agent = nullptr;
};

}  // namespace hdmap 
}  // namespace apollo

#endif  // MODULES_MAP_TOOLS_MAP_DATACHECKER_WORKER_AGENT_H
