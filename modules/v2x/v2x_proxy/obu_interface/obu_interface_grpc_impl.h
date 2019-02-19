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
 * @file obu_interface_grpc_impl.h
 * @brief define v2x proxy module and onboard unit interface grpc impl class
 */

#pragma once

#include <memory>

#include "modules/v2x/v2x_proxy/obu_interface/grpc_interface/grpc_client.h"
#include "modules/v2x/v2x_proxy/obu_interface/grpc_interface/grpc_server.h"
#include "modules/v2x/v2x_proxy/obu_interface/obu_interface_abstract_class.h"

namespace apollo {
namespace v2x {

class ObuInterFaceGrpcImpl : public ObuInterFaceBase {
 public:
  ObuInterFaceGrpcImpl();
  ~ObuInterFaceGrpcImpl();

  /* function that init grpc server
   */
  bool InitialServer() override;

  /* function that init grpc client
   */
  bool InitialClient() override;

  /* function that get perception obstacles through grpc
  @param output return latest v2x perception obstacles
  */
  void GetV2xObstaclesFromObu(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg)
      override;

  /* function that get v2x traffic light through grpc
  @param output return latest v2x traffic light
  */
  void GetV2xTrafficLightFromObu(
      const std::shared_ptr<IntersectionTrafficLightData> &msg) override;

  /* function that send car status through grpc
  @param input send car status msg to grpc
  */
  void SendCarStatusToObu(const std::shared_ptr<CarStatus> &msg) override;

  /* function that send perception obstacles through grpc
  @param input send perception obstacles msg to grpc
  */
  void SendObstaclesToObu(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg)
      override;

  /* function that return init flag
   */
  bool InitFlag() { return init_succ_; }

 private:
  /* thread function that run server
   */
  void ThreadRunServer();
  std::shared_ptr<GrpcClientImpl> grpc_client_;
  std::shared_ptr<GrpcServerImpl> grpc_server_;
  std::unique_ptr<grpc::Server> server_;

  bool grpc_client_init_flag_ = false;
  bool grpc_server_init_flag_ = false;

  bool init_succ_ = false;
  bool exit_flag_ = false;
  std::unique_ptr<std::thread> thread_ptr_;
  std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace v2x
}  // namespace apollo
