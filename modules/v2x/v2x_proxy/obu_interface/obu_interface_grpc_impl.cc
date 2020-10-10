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

#include <memory>
#include <string>
#include <utility>

#include "modules/v2x/common/v2x_proxy_gflags.h"

namespace apollo {
namespace v2x {

using ::apollo::v2x::CarStatus;
using ::apollo::v2x::obu::ObuRsi;
using ::apollo::v2x::obu::ObuTrafficLight;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;

ObuInterFaceGrpcImpl::ObuInterFaceGrpcImpl()
    : grpc_client_(new GrpcClientImpl(grpc::CreateChannel(
          FLAGS_grpc_client_host + ":" + FLAGS_grpc_client_port,
          grpc::InsecureChannelCredentials()))) {
  AINFO << "ObuInterFaceGrpcImpl Start Construct.";
  cli_init_ = grpc_client_->InitFlag();
  grpc_server_.reset(new GrpcServerImpl());
  srv_init_ = grpc_server_->InitFlag();
  CHECK(InitialClient());
  CHECK(InitialServer());
  init_succ_ = true;
}

ObuInterFaceGrpcImpl::~ObuInterFaceGrpcImpl() {
  {
    std::unique_lock<std::mutex> lck(mutex_);
    exit_flag_ = true;
  }
  condition_.notify_all();
  if (!!thread_grpc_ && thread_grpc_->joinable()) {
    thread_grpc_->join();
  }
  AINFO << "close obu interface success";
}

bool ObuInterFaceGrpcImpl::InitialClient() { return cli_init_; }

bool ObuInterFaceGrpcImpl::InitialServer() {
  if (!srv_init_) {
    return false;
  }
  thread_grpc_.reset(new std::thread([this]() { this->ThreadRunServer(); }));
  return thread_grpc_ != nullptr;
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
}

void ObuInterFaceGrpcImpl::GetV2xTrafficLightFromObu(
    std::shared_ptr<ObuTrafficLight> *msg) {
  grpc_server_->GetMsgFromGrpc(msg);
}

void ObuInterFaceGrpcImpl::GetV2xObstaclesFromObu(
    std::shared_ptr<::apollo::v2x::V2XObstacles> *msg) {
  grpc_server_->GetMsgFromGrpc(msg);
}

void ObuInterFaceGrpcImpl::GetV2xRsiFromObu(std::shared_ptr<ObuRsi> *msg) {
  grpc_server_->GetMsgFromGrpc(msg);
}

void ObuInterFaceGrpcImpl::SendCarStatusToObu(
    const std::shared_ptr<CarStatus> &msg) {
  grpc_client_->SendMsgToGrpc(msg);
}
}  // namespace v2x
}  // namespace apollo
