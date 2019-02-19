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
 * @file grpc_server.h
 * @brief define v2x proxy module and onboard unit interface grpc implement
 */

#pragma once

#include <memory>

#include "cyber/cyber.h"
#include "modules/v2x/proto/v2x_service_obu_to_car.grpc.pb.h"

namespace apollo {
namespace v2x {

class GrpcServerImpl final : public ObuToCar::Service {
 public:
  /* construct function
   */
  GrpcServerImpl();

  ~GrpcServerImpl() {}

  bool InitFlag() { return init_flag_; }

  /* function that send perception obstacles through grpc
  @param input perception obstacles grpc request
  @param output grpc response
  */
  grpc::Status SendPerceptionObstacles(
      grpc::ServerContext* context,
      const apollo::perception::PerceptionObstacles* request,
      StatusResponse* response);

  /* function that send v2x trafficlight through grpc
  @param input v2x trafficlight grpc request
  @param output grpc response
  */
  grpc::Status SendV2xTrafficLight(grpc::ServerContext* context,
                                   const IntersectionTrafficLightData* request,
                                   StatusResponse* response);

  /* function that get latest msg from grpc
  @param output shared_ptr
  */
  void GetMsgFromGrpc(
      const std::shared_ptr<apollo::perception::PerceptionObstacles>& ptr);
  void GetMsgFromGrpc(const std::shared_ptr<IntersectionTrafficLightData>& ptr);

 private:
  std::mutex traffic_light_mutex_;
  std::mutex obstacles_mutex_;
  apollo::perception::PerceptionObstacles latest_obstacles_;
  IntersectionTrafficLightData latest_trafficlight_;
  bool init_flag_ = false;
  bool first_recv_flag_ = true;
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Writer<StatusResponse>> first_flag_writer_ = nullptr;
};

}  // namespace v2x
}  // namespace apollo
