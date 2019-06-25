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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_ALIGNMENT_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_ALIGNMENT_H
#include <grpc++/grpc++.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <typeinfo>
#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client_common.h"
#include "modules/map/tools/map_datachecker/client/client_gflags.h"
#include "modules/map/tools/map_datachecker/client/exception_handler.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"
namespace apollo {
namespace hdmap {

using STATIC_REQUEST_TYPE = apollo::hdmap::StaticAlignRequest;
using STATIC_RESPONSE_TYPE = apollo::hdmap::StaticAlignResponse;
using EIGHTROUTE_REQUEST_TYPE = apollo::hdmap::EightRouteRequest;
using EIGHTROUTE_RESPONSE_TYPE = apollo::hdmap::EightRouteResponse;

template <typename REQUEST_TYPE, typename RESPONSE_TYPE>
class Alignment {
 public:
  Alignment() {
    YAML::Node node = YAML::LoadFile(FLAGS_client_conf_yaml);
    std::string server_addr =
        node["grpc_host_port"]["grpc_host"].as<std::string>() + ":" +
        node["grpc_host_port"]["grpc_port"].as<std::string>();
    _check_period = node["alignment"]["check_period"].as<int>();
    _service_stub = CollectionCheckerService::NewStub(
        grpc::CreateChannel(server_addr, grpc::InsecureChannelCredentials()));
  }

  int sync_start() {
    int ret = _start();
    if (ret != 0) {
      AERROR << "start alignment failed";
      return -1;
    }
    return periodic_check();
  }

  int sync_stop() {
    // stop server
    return _stop();
  }

  int periodic_check() {
    int ret = 0;
    while (true) {
      double progress = 0.0;
      ret = _check(&progress);
      if (ret != 0) {
        AERROR << "alignment check failed";
        break;
      }
      AINFO << "alignment check progress: [" << progress << "]";
      fprintf(USER_STREAM, "alignment progress: %.2lf%%\n", progress * 100);
      if (fabs(progress - 1.0) < 1e-8) {
        AINFO << "diff " << std::to_string(fabs(progress - 1.0));
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(_check_period));
    }
    if (ret != 0) {
      return -1;
    }
    ret = _stop();
    if (ret != 0) {
      AERROR << "alignment stop failed";
      return -1;
    }
    return 0;
  }

  int grpc_stub(REQUEST_TYPE* request, RESPONSE_TYPE* response) {
    grpc::Status status = grpc_alignment_stub(request, response);
    if (status.error_code() == grpc::StatusCode::UNAVAILABLE) {
      AERROR << "FATAL Error. Map grpc service is UNAVAILABLE.";
      fprintf(USER_STREAM, "You should start server first\n");
      return -1;
    }
    AINFO << "response error code: " << response->code();
    if (response->code() != ErrorCode::SUCCESS) {
      return ExceptionHandler::exception_handler(response->code());
    }
    return 0;
  }

 private:
  int _start() {
    REQUEST_TYPE request;
    request.set_cmd(CmdType::START);
    AINFO << "alignment request: "
          << "cmd: [" << request.cmd() << "]";
    RESPONSE_TYPE response;
    return grpc_stub(&request, &response);
  }

  int _check(double* progress) {
    REQUEST_TYPE request;
    request.set_cmd(CmdType::CHECK);
    AINFO << "alignment request: "
          << "cmd: [" << request.cmd() << "]";
    RESPONSE_TYPE response;
    int ret = grpc_stub(&request, &response);
    *progress = response.progress();
    return ret;
  }

  int _stop() {
    REQUEST_TYPE request;
    request.set_cmd(CmdType::STOP);
    AINFO << "alignment request: "
          << "cmd: [" << request.cmd() << "]";
    RESPONSE_TYPE response;
    return grpc_stub(&request, &response);
  }

 private:
  int _check_period;

 protected:
  std::unique_ptr<CollectionCheckerService::Stub> _service_stub;
  virtual grpc::Status grpc_alignment_stub(REQUEST_TYPE* request,
                                           RESPONSE_TYPE* response) = 0;
};

class StaticAlign
    : public Alignment<STATIC_REQUEST_TYPE, STATIC_RESPONSE_TYPE> {
  grpc::Status grpc_alignment_stub(STATIC_REQUEST_TYPE* request,
                                   STATIC_RESPONSE_TYPE* response) {
    grpc::ClientContext context;
    return _service_stub->StaticAlign(&context, *request, response);
  }
};

class EightRoute
    : public Alignment<EIGHTROUTE_REQUEST_TYPE, EIGHTROUTE_RESPONSE_TYPE> {
  grpc::Status grpc_alignment_stub(EIGHTROUTE_REQUEST_TYPE* request,
                                   EIGHTROUTE_RESPONSE_TYPE* response) {
    grpc::ClientContext context;
    return _service_stub->EightRoute(&context, *request, response);
  }
};

}  // namespace hdmap
}  // namespace apollo
#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_ALIGNMENT_H
