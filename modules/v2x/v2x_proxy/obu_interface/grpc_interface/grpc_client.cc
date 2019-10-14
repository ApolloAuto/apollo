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

#include "cyber/common/log.h"

namespace apollo {
namespace v2x {

using apollo::perception::PerceptionObstacles;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

GrpcClientImpl::GrpcClientImpl(std::shared_ptr<Channel> channel)
    : stub_(CarToObu::NewStub(channel)) {
  AINFO << "GrpcClientImpl initial success";
  carstatus_tv_nsec_ =
      (1000000000 / static_cast<int>(FLAGS_v2x_carstatus_timer_frequency) / 2);
  init_flag_ = true;
}

void GrpcClientImpl::SendMsgToGrpc(const std::shared_ptr<CarStatus> &msg) {
  // verify CarStatus msg valid
  if (!msg->has_localization() /*  || !msg->has_chassis_detail() */) {
    AERROR << "SendCarStatusToObu msg is not valid";
    return;
  }
  // set timeout
  ClientContext context;
  gpr_timespec timespec;
  timespec.tv_sec = 0;
  timespec.tv_nsec = carstatus_tv_nsec_;  // 80000000; // 80ms
  timespec.clock_type = GPR_TIMESPAN;
  context.set_deadline(timespec);
  UpdateStatus response;

  // time used statistics
  auto start = std::chrono::steady_clock::now();
  Status status = stub_->PushCarStatus(&context, *msg, &response);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = end - start;

  // response check: error_code 4: time out; 0: success;
  if (status.error_code() != 0) {
    AERROR << "stub PushCarStatus error : code " << status.error_code();
  } else {
    ADEBUG << "stub PushCarStatus success. time used: "
           << time_used.count() * 1000 << "ms"
           << "error : code " << status.error_code();
  }
}

void GrpcClientImpl::SendMsgToGrpc(
    const std::shared_ptr<PerceptionObstacles> &msg) {
  // verify perception obstacles msg valid
  if (msg->perception_obstacle().empty()) {
    AERROR << "SendObstaclesToObu msg is not valid";
    return;
  }

  // set timeout
  ClientContext context;
  gpr_timespec timespec;
  timespec.tv_sec = 0;
  timespec.tv_nsec = 80000000;  // 80ms
  timespec.clock_type = GPR_TIMESPAN;
  context.set_deadline(timespec);
  UpdateStatus response;

  // time used statistics
  auto start = std::chrono::steady_clock::now();
  Status status = stub_->PushPerceptionResult(&context, *msg, &response);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = end - start;

  // response check: error_code 4: time out; 0: success;
  if (status.error_code() != 0) {
    AERROR << "stub PushPerceptionResult error : code " << status.error_code();
  } else {
    ADEBUG << "stub PushPerceptionResult success. time used: "
           << time_used.count() * 1000 << "ms";
  }
}

}  // namespace v2x
}  // namespace apollo
