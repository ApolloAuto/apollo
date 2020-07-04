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
 * @file obu_interface_grpc_impl.cc
 * @brief define v2x proxy module and onboard unit interface grpc impl class
 */
#include "modules/v2x/v2x_proxy/obu_interface/obu_interface_grpc_impl.h"

#include <string>
#include <utility>

namespace apollo {
namespace v2x {

using apollo::perception::PerceptionObstacles;
using grpc::ServerBuilder;

ObuInterFaceGrpcImpl::ObuInterFaceGrpcImpl() {
  AINFO << "ObuInterFaceGrpcImpl Start Construct.";
  grpc_client_ = std::make_shared<GrpcClientImpl>(
      grpc::CreateChannel(FLAGS_grpc_client_host + ":" + FLAGS_grpc_client_port,
                          grpc::InsecureChannelCredentials()));
  grpc_client_init_flag_ = grpc_client_->InitFlag();
  bool res_client = InitialClient();
  ACHECK(res_client) << "ObuInterFaceGrpcImpl grpc client initial failed";
  grpc_server_ = std::make_shared<GrpcServerImpl>();
  grpc_server_init_flag_ = grpc_server_->InitFlag();
  bool res_server = InitialServer();
  ACHECK(res_server) << "ObuInterFaceGrpcImpl grpc server initial failed";
  init_succ_ = true;
}

ObuInterFaceGrpcImpl::~ObuInterFaceGrpcImpl() {
  // if (server_ != nullptr) {
  //     server_->Shutdown();
  // }
  if (thread_ptr_ == nullptr) {
    AINFO << "close obu interface success";
    return;
  }
  {
    std::unique_lock<std::mutex> lck(mutex_);
    exit_flag_ = true;
  }
  condition_.notify_all();
  thread_ptr_->join();
  AINFO << "close obu interface success";
}

bool ObuInterFaceGrpcImpl::InitialClient() { return grpc_client_init_flag_; }

bool ObuInterFaceGrpcImpl::InitialServer() {
  if (!grpc_server_init_flag_) {
    return false;
  }

  thread_ptr_.reset(
      new std::thread(&ObuInterFaceGrpcImpl::ThreadRunServer, this));
  return (thread_ptr_ != nullptr);
}

void ObuInterFaceGrpcImpl::ThreadRunServer() {
  std::unique_lock<std::mutex> lck(mutex_);
  auto start = std::chrono::steady_clock::now();
  std::string server_address(FLAGS_grpc_server_host + ":" +
                             FLAGS_grpc_server_port);
  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(grpc_server_.get());
  // Finally assemble the server.
  auto tmp = builder.BuildAndStart();
  server_ = std::move(tmp);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = end - start;
  AINFO << "ObuInterFaceGrpcImpl grpc server has listening on : "
        << server_address << " time used : " << time_used.count();
  condition_.wait(lck, [&]() { return exit_flag_; });
  // server_->Wait();
  // cyber::Spin();
}

void ObuInterFaceGrpcImpl::GetV2xObstaclesFromObu(
    const std::shared_ptr<PerceptionObstacles> &msg) {
  grpc_server_->GetMsgFromGrpc(msg);
}

void ObuInterFaceGrpcImpl::GetV2xTrafficLightFromObu(
    const std::shared_ptr<IntersectionTrafficLightData> &msg) {
  grpc_server_->GetMsgFromGrpc(msg);
}
void ObuInterFaceGrpcImpl::SendCarStatusToObu(
    const std::shared_ptr<CarStatus> &msg) {
  grpc_client_->SendMsgToGrpc(msg);
}
void ObuInterFaceGrpcImpl::SendObstaclesToObu(
    const std::shared_ptr<PerceptionObstacles> &msg) {
  grpc_client_->SendMsgToGrpc(msg);
}

}  // namespace v2x
}  // namespace apollo
