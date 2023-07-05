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
#pragma once

#include <memory>
#include <string>
#include <typeinfo>

#include "grpc++/grpc++.h"
#include "yaml-cpp/yaml.h"

#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client_common.h"
#include "modules/map/tools/map_datachecker/client/client_gflags.h"
#include "modules/map/tools/map_datachecker/client/exception_handler.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"

namespace apollo {
namespace hdmap {

template <typename REQUEST_TYPE, typename RESPONSE_TYPE>
class Alignment {
 public:
  Alignment() {
    YAML::Node node = YAML::LoadFile(FLAGS_client_conf_yaml);
    std::string server_addr =
        node["grpc_host_port"]["grpc_host"].as<std::string>() + ":" +
        node["grpc_host_port"]["grpc_port"].as<std::string>();
    check_period_ = node["alignment"]["check_period"].as<int>();
    service_stub_ = CollectionCheckerService::NewStub(
        grpc::CreateChannel(server_addr, grpc::InsecureChannelCredentials()));
  }

  int SyncStart() {
    int ret = Start();
    if (ret != 0) {
      AERROR << "start alignment failed";
      return -1;
    }
    return PeriodicCheck();
  }

  int SyncStop() {
    // stop server
    return Stop();
  }

  int PeriodicCheck() {
    int ret = 0;
    while (true) {
      double progress = 0.0;
      ret = Check(&progress);
      if (ret != 0) {
        AERROR << "alignment check failed";
        break;
      }
      AINFO << "alignment check progress: [" << progress << "]";
      fprintf(USER_STREAM, "alignment progress: %.2lf%%\n", progress * 100);
      if (fabs(progress - 1.0) < 1e-8) {
        AINFO << "diff " << fabs(progress - 1.0);
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(check_period_));
    }
    if (ret != 0) {
      return -1;
    }
    ret = Stop();
    if (ret != 0) {
      AERROR << "alignment stop failed";
      return -1;
    }
    return 0;
  }

  int GrpcStub(REQUEST_TYPE* request, RESPONSE_TYPE* response) {
    grpc::Status status = GrpcAlignmentStub(request, response);
    if (status.error_code() == grpc::StatusCode::UNAVAILABLE) {
      AERROR << "FATAL Error. Map grpc service is UNAVAILABLE.";
      fprintf(USER_STREAM, "You should start server first\n");
      return -1;
    }
    AINFO << "response error code: " << response->code();
    if (response->code() != ErrorCode::SUCCESS) {
      return ExceptionHandler::ExceptionHandlerFun(response->code());
    }
    return 0;
  }

 private:
  int Start() {
    REQUEST_TYPE request;
    request.set_cmd(CmdType::START);
    AINFO << "alignment request: "
          << "cmd: [" << request.cmd() << "]";
    RESPONSE_TYPE response;
    return GrpcStub(&request, &response);
  }

  int Check(double* progress) {
    REQUEST_TYPE request;
    request.set_cmd(CmdType::CHECK);
    AINFO << "alignment request: "
          << "cmd: [" << request.cmd() << "]";
    RESPONSE_TYPE response;
    int ret = GrpcStub(&request, &response);
    *progress = response.progress();
    return ret;
  }

  int Stop() {
    REQUEST_TYPE request;
    request.set_cmd(CmdType::STOP);
    AINFO << "alignment request: "
          << "cmd: [" << request.cmd() << "]";
    RESPONSE_TYPE response;
    return GrpcStub(&request, &response);
  }

 private:
  int check_period_;

 protected:
  std::unique_ptr<CollectionCheckerService::Stub> service_stub_;
  virtual grpc::Status GrpcAlignmentStub(REQUEST_TYPE* request,
                                         RESPONSE_TYPE* response) = 0;
};

class StaticAlign : public Alignment<StaticAlignRequest, StaticAlignResponse> {
  grpc::Status GrpcAlignmentStub(StaticAlignRequest* request,
                                 StaticAlignResponse* response) {
    grpc::ClientContext context;
    return service_stub_->ServiceStaticAlign(&context, *request, response);
  }
};

class EightRoute : public Alignment<EightRouteRequest, EightRouteResponse> {
  grpc::Status GrpcAlignmentStub(EightRouteRequest* request,
                                 EightRouteResponse* response) {
    grpc::ClientContext context;
    return service_stub_->ServiceEightRoute(&context, *request, response);
  }
};

}  // namespace hdmap
}  // namespace apollo
