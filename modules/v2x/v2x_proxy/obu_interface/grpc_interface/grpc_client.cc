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
 * @file grpc_client.cc
 * @brief define v2x proxy module and onboard unit interface grpc implement
 */

#include "modules/v2x/v2x_proxy/obu_interface/grpc_interface/grpc_client.h"

#include <chrono>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace apollo {
namespace v2x {

using ::apollo::v2x::CarStatus;
using ::apollo::v2x::UpdateStatus;
using ::apollo::v2x::obu::ObuTrafficLight;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

GrpcClientImpl::GrpcClientImpl(std::shared_ptr<Channel> channel)
    : stub_(::apollo::v2x::CarToObu::NewStub(channel)) {
  AINFO << "GrpcClientImpl initial success";
  car_status_tv_nsec_ = (1000000000 / FLAGS_v2x_car_status_timer_frequency / 2);
  init_flag_ = true;
}

void GrpcClientImpl::SendMsgToGrpc(const std::shared_ptr<CarStatus> &msg) {
  if (!msg->has_localization()) {
    return;
  }
  // set timeout
  ClientContext context;
  gpr_timespec timespec;
  timespec.tv_sec = 0;
  timespec.tv_nsec = car_status_tv_nsec_;  // 80000000; // 80ms
  timespec.clock_type = GPR_TIMESPAN;
  context.set_deadline(timespec);
  UpdateStatus response;

  // time used statistics
  auto start = std::chrono::steady_clock::now();
  Status status = stub_->PushCarStatus(&context, *msg, &response);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = end - start;

  // response check: error_code 4: time out; 0: success;
  AINFO << "stub PushCarStatus Time used: " << time_used.count() * 1000
        << " ms";
}

}  // namespace v2x
}  // namespace apollo
