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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <grpc++/grpc++.h>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/v2x/proto/v2x_obu_rsi.pb.h"
#include "modules/v2x/proto/v2x_service_obu_to_car.grpc.pb.h"
#include "modules/v2x/proto/v2x_traffic_light.pb.h"

#include "cyber/cyber.h"

namespace apollo {
namespace v2x {

class GrpcServerImpl final : public ::apollo::v2x::ObuToCar::Service {
 public:
  /* construct function
   */
  GrpcServerImpl();

  ~GrpcServerImpl() {
    traffic_light_condition_.notify_all();
    rsi_condition_.notify_all();
  }

  bool InitFlag() { return init_flag_; }

  /* function that send v2x traffic_light through grpc
  @param input v2x traffic_light grpc request
  @param output grpc response
  */
  grpc::Status SendV2xTrafficLight(
      grpc::ServerContext *context,
      const ::apollo::v2x::obu::ObuTrafficLight *request,
      ::apollo::v2x::StatusResponse *response) override;

  /* function that send v2x monitor messages through grpc
  @param input v2x monitor grpc request
  @param output grpc response
  */
  grpc::Status SendObuAlarm(grpc::ServerContext *context,
                            const ::apollo::v2x::ObuAlarm *request,
                            ::apollo::v2x::StatusResponse *response) override;

  /* function that send perception obstacles through grpc
     @param input perception obstacles grpc request
     @param output grpc response
    */
  grpc::Status SendPerceptionObstacles(grpc::ServerContext *context,
                                       const apollo::v2x::V2XObstacles *request,
                                       StatusResponse *response) override;

  grpc::Status SendV2xRSI(grpc::ServerContext *context,
                          const ::apollo::v2x::obu::ObuRsi *request,
                          ::apollo::v2x::StatusResponse *response);

  void GetMsgFromGrpc(
      std::shared_ptr<::apollo::v2x::obu::ObuTrafficLight> *ptr);

  void GetMsgFromGrpc(std::shared_ptr<::apollo::v2x::obu::ObuRsi> *ptr);
  void GetMsgFromGrpc(std::shared_ptr<::apollo::v2x::V2XObstacles> *ptr);

 private:
  std::mutex traffic_light_mutex_;
  std::mutex rsi_mutex_;
  std::mutex obstacles_mutex_;
  std::condition_variable traffic_light_condition_;
  std::condition_variable rsi_condition_;
  std::condition_variable obs_condition_;
  ::apollo::v2x::obu::ObuTrafficLight latest_traffic_light_;
  ::apollo::v2x::obu::ObuRsi latest_rsi_;
  ::apollo::v2x::V2XObstacles latest_obstacles_;

  bool init_flag_ = false;
  bool refresh_ = false;
  bool rsi_refresh_ = false;
  bool obs_refresh_ = false;
  std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
};

}  // namespace v2x
}  // namespace apollo
