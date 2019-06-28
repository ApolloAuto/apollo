/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <grpc++/grpc++.h>
#include <memory>
#include <utility>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.pb.h"
#include "modules/map/tools/map_datachecker/server/alignment_agent.h"
#include "modules/map/tools/map_datachecker/server/channel_verify_agent.h"
#include "modules/map/tools/map_datachecker/server/common.h"
#include "modules/map/tools/map_datachecker/server/loops_verify_agent.h"
#include "modules/map/tools/map_datachecker/server/pose_collection.h"

namespace apollo {
namespace hdmap {

class MapDataCheckerAgent final
    : public std::enable_shared_from_this<MapDataCheckerAgent>,
      public CollectionCheckerService::Service {
 public:
  using STATIC_ALIGN_AGENT_TYPE =
      AlignmentAgent<StaticAlign, StaticAlignRequest, StaticAlignResponse>;
  using EIGHT_ROUTE_AGENT_TYPE =
      AlignmentAgent<EightRoute, EightRouteRequest, EightRouteResponse>;

 public:
  MapDataCheckerAgent();
  inline std::shared_ptr<MapDataCheckerAgent> GetWorkerAgent() {
    return shared_from_this();
  }
  std::shared_ptr<PoseCollectionAgent> GetSpPoseCollectionAgent();

  grpc::Status ServiceChannelVerify(grpc::ServerContext *context,
                                    ChannelVerifyRequest *request,
                                    ChannelVerifyResponse *response);
  grpc::Status ServiceStaticAlign(grpc::ServerContext *context,
                                  StaticAlignRequest *request,
                                  StaticAlignResponse *response);
  grpc::Status ServiceEightRoute(grpc::ServerContext *context,
                                 EightRouteRequest *request,
                                 EightRouteResponse *response);
  grpc::Status ServiceLoopsVerify(grpc::ServerContext *context,
                                  LoopsVerifyRequest *request,
                                  LoopsVerifyResponse *response);

 private:
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  std::shared_ptr<PoseCollectionAgent> sp_pose_collection_agent_ = nullptr;
  std::shared_ptr<ChannelVerifyAgent> sp_channel_checker_agent_ = nullptr;
  std::shared_ptr<STATIC_ALIGN_AGENT_TYPE> sp_static_align_agent_ = nullptr;
  std::shared_ptr<EIGHT_ROUTE_AGENT_TYPE> sp_eight_route_agent_ = nullptr;
  std::shared_ptr<LoopsVerifyAgent> sp_loops_verify_agent_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo
