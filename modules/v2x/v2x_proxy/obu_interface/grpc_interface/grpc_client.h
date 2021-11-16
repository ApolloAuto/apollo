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
 * @file grpc_client.h
 * @brief define v2x proxy module and onboard unit interface grpc implement
 */

#pragma once

#include <memory>
#include <mutex>

#include <grpc++/grpc++.h>

#include "modules/v2x/proto/v2x_car_status.pb.h"
#include "modules/v2x/proto/v2x_obu_traffic_light.pb.h"
#include "modules/v2x/proto/v2x_service_car_to_obu.grpc.pb.h"

#include "cyber/cyber.h"
#include "modules/v2x/common/v2x_proxy_gflags.h"

namespace apollo {
namespace v2x {

class GrpcClientImpl {
 public:
  /* construct function
  @param input car_status type msg shared ptr
  */
  explicit GrpcClientImpl(std::shared_ptr<grpc::Channel> channel);

  ~GrpcClientImpl() {}

  bool InitFlag() { return init_flag_; }

  /*function that send car status msg through grpc
  @param input car_status type msg shared ptr
  */
  void SendMsgToGrpc(const std::shared_ptr<::apollo::v2x::CarStatus> &msg);

 private:
  //  grpc service stub
  std::unique_ptr<::apollo::v2x::CarToObu::Stub> stub_;
  int car_status_tv_nsec_ = 0;
  bool init_flag_ = false;
};

}  // namespace v2x
}  // namespace apollo
