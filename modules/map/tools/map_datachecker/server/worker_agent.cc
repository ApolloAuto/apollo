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
#include "modules/map/tools/map_datachecker/server/worker_agent.h"

#include <memory>

#include "grpc++/grpc++.h"

namespace apollo {
namespace hdmap {

MapDataCheckerAgent::MapDataCheckerAgent() {
  sp_conf_ = ParseJson(FLAGS_conf_json);
  assert(sp_conf_ != nullptr);
  sp_pose_collection_agent_ = std::make_shared<PoseCollectionAgent>(sp_conf_);
  sp_channel_checker_agent_ = std::make_shared<ChannelVerifyAgent>(sp_conf_);

  sp_static_align_agent_ = std::make_shared<STATIC_ALIGN_AGENT_TYPE>(
      sp_conf_, sp_pose_collection_agent_);
  sp_eight_route_agent_ = std::make_shared<EIGHT_ROUTE_AGENT_TYPE>(
      sp_conf_, sp_pose_collection_agent_);
  sp_loops_verify_agent_ =
      std::make_shared<LoopsVerifyAgent>(sp_conf_, sp_pose_collection_agent_);
  AINFO << "MapDataCheckerAgent create successfully";
}

std::shared_ptr<PoseCollectionAgent>
MapDataCheckerAgent::GetSpPoseCollectionAgent() {
  return sp_pose_collection_agent_;
}

grpc::Status MapDataCheckerAgent::ServiceChannelVerify(
    grpc::ServerContext *context, ChannelVerifyRequest *request,
    ChannelVerifyResponse *response) {
  return sp_channel_checker_agent_->ProcessGrpcRequest(context, request,
                                                       response);
}

grpc::Status MapDataCheckerAgent::ServiceStaticAlign(
    grpc::ServerContext *context, StaticAlignRequest *request,
    StaticAlignResponse *response) {
  return sp_static_align_agent_->ProcessGrpcRequest(context, request, response);
}

grpc::Status MapDataCheckerAgent::ServiceEightRoute(
    grpc::ServerContext *context, EightRouteRequest *request,
    EightRouteResponse *response) {
  return sp_eight_route_agent_->ProcessGrpcRequest(context, request, response);
}

grpc::Status MapDataCheckerAgent::ServiceLoopsVerify(
    grpc::ServerContext *context, LoopsVerifyRequest *request,
    LoopsVerifyResponse *response) {
  return sp_loops_verify_agent_->ProcessGrpcRequest(context, request, response);
}

}  // namespace hdmap
}  // namespace apollo
