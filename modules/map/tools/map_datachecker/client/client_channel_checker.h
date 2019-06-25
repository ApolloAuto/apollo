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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_CHANNEL_CHECHER_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_CHANNEL_CHECHER_H
#include <memory>
#include <string>
#include "modules/map/tools/map_datachecker/client/client_gflags.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"
namespace apollo {
namespace hdmap {

using CHANNEL_VERIFY_REQUEST_TYPE = apollo::hdmap::ChannelVerifyRequest;
using CHANNEL_VERIFY_RESPONSE_TYPE = apollo::hdmap::ChannelVerifyResponse;

class ChannelChecker {
 public:
  ChannelChecker(const std::string& stop_flag_file);
  int sync_start(std::string& record_path);
  int sync_stop();
  int periodic_check();
  int grpc_stub(CHANNEL_VERIFY_REQUEST_TYPE& request,
                CHANNEL_VERIFY_RESPONSE_TYPE& response);

 private:
  int start(std::string& record_path);
  int check();
  int stop();
  int process_abnormal(CHANNEL_VERIFY_RESPONSE_TYPE& response);

 private:
  std::unique_ptr<CollectionCheckerService::Stub> _service_stub;
  int _check_period;
  const std::string& _stop_flag_file;
};

}  // namespace hdmap
}  // namespace apollo
#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_CHANNEL_CHECHER_H
