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

#include "modules/map/tools/map_datachecker/client/client_gflags.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"

namespace apollo {
namespace hdmap {

class ChannelChecker {
 public:
  explicit ChannelChecker(const std::string& stop_flag_file);
  int SyncStart(const std::string& record_path);
  int SyncStop();
  int PeriodicCheck();
  int GrpcStub(ChannelVerifyRequest* request, ChannelVerifyResponse* response);

 private:
  int Start(const std::string& record_path);
  int Check();
  int Stop();
  int ProcessAbnormal(ChannelVerifyResponse* response);

 private:
  std::unique_ptr<CollectionCheckerService::Stub> service_stub_;
  int check_period_;
  const std::string& stop_flag_file_;
};

}  // namespace hdmap
}  // namespace apollo
