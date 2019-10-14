/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file grpc_server.cc
 * @brief define v2x proxy module and onboard unit interface grpc implement
 */

#include "modules/v2x/v2x_proxy/obu_interface/grpc_interface/grpc_server.h"

namespace apollo {
namespace v2x {

using apollo::perception::PerceptionObstacles;
using grpc::Status;

GrpcServerImpl::GrpcServerImpl() : node_(cyber::CreateNode("v2x_grpc_server")) {
  CHECK(node_) << "Create v2x grpc server node failed";
  first_flag_writer_ =
      node_->CreateWriter<StatusResponse>("/apollo/v2x/inner/sync_flag");
  CHECK(first_flag_writer_) << "Create sync flag writer failed";
  AINFO << "GrpcServerImpl initial success";
  init_flag_ = true;
}

grpc::Status GrpcServerImpl::SendPerceptionObstacles(
    grpc::ServerContext* /* context */, const PerceptionObstacles* request,
    StatusResponse* response) {
  ADEBUG << "Received SendPerceptionObstacles request from client! \n";
  if (request->perception_obstacle().empty()) {
    response->set_status(false);
    response->set_info("error perception obstacle size == 0");
    AERROR << "SendPerceptionObstacles request has no perception obstacle";
    return Status::CANCELLED;
  }
  if (!request->has_header()) {
    response->set_status(false);
    response->set_info("error no header in PerceptionObstacles request");
    AERROR << "SendPerceptionObstacles request has no header";
    return Status::CANCELLED;
  }
  {
    std::lock_guard<std::mutex> guard(obstacles_mutex_);
    latest_obstacles_.CopyFrom(*request);
  }
  response->set_status(true);
  ADEBUG << "SendPerceptionObstacles response success.";
  return Status::OK;
}

grpc::Status GrpcServerImpl::SendV2xTrafficLight(
    grpc::ServerContext* /* context */,
    const IntersectionTrafficLightData* request, StatusResponse* response) {
  ADEBUG << "Received SendV2xTrafficLight request from client! \n";
  if (request->current_lane_trafficlight().single_traffic_light().empty()) {
    response->set_status(false);
    response->set_info("error v2x traffic light size == 0");
    AERROR << "SendV2xTrafficLight request has no traffic light";
    return Status::CANCELLED;
  }
  if (!request->has_header()) {
    response->set_status(false);
    response->set_info(
        "error no header in IntersectionTrafficLightData request");
    AERROR << "SendV2xTrafficLight request has no header";
    return Status::CANCELLED;
  }
  {
    std::lock_guard<std::mutex> guard(traffic_light_mutex_);
    latest_trafficlight_.CopyFrom(*request);
  }
  if (first_recv_flag_) {
    auto msg = std::make_shared<StatusResponse>();
    msg->set_status(true);
    if (first_flag_writer_->Write(msg)) {
      AINFO << "grpc sync flag send success";
      first_recv_flag_ = false;
    } else {
      AERROR << "grpc sync flag send failed";
    }
  }
  response->set_status(true);
  ADEBUG << "SendV2xTrafficLight response success.";
  return Status::OK;
}

void GrpcServerImpl::GetMsgFromGrpc(
    const std::shared_ptr<PerceptionObstacles>& ptr) {
  if (latest_obstacles_.perception_obstacle().empty()) {
    AERROR << "GetMsgFromGrpc PerceptionObstacles is invalid";
    return;
  }
  if (!latest_obstacles_.has_header()) {
    AERROR << "GetMsgFromGrpc PerceptionObstacles is invalid";
    return;
  }
  std::lock_guard<std::mutex> guard(obstacles_mutex_);
  ptr->CopyFrom(latest_obstacles_);
}

void GrpcServerImpl::GetMsgFromGrpc(
    const std::shared_ptr<IntersectionTrafficLightData>& ptr) {
  if (!latest_trafficlight_.has_current_lane_trafficlight()) {
    AERROR << "GetMsgFromGrpc IntersectionTrafficLightData is invalid";
    return;
  }
  if (!latest_trafficlight_.has_header()) {
    AERROR << "GetMsgFromGrpc IntersectionTrafficLightData is invalid";
    return;
  }
  std::lock_guard<std::mutex> guard(traffic_light_mutex_);
  ptr->CopyFrom(latest_trafficlight_);
}

}  // namespace v2x
}  // namespace apollo
