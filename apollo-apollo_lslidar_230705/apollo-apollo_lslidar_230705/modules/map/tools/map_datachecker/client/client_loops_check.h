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
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "modules/map/tools/map_datachecker/client/client_common.h"
#include "modules/map/tools/map_datachecker/client/client_gflags.h"
#include "modules/map/tools/map_datachecker/client/exception_handler.h"
#include "modules/map/tools/map_datachecker/proto/collection_service.grpc.pb.h"

namespace apollo {
namespace hdmap {

class LoopsChecker {
 public:
  explicit LoopsChecker(const std::string& time_flag_file);
  int SyncStart(bool* reached);

 private:
  std::vector<std::pair<double, double>> GetTimeRanges();
  int PeriodicCheck(bool* reached);
  int GrpcStub(LoopsVerifyRequest* request, LoopsVerifyResponse* response);
  int Start(const std::vector<std::pair<double, double>>& time_ranges);
  int Check(double* progress, bool* reached);
  int Stop();

 private:
  std::unique_ptr<CollectionCheckerService::Stub> service_stub_;
  const std::string& time_flag_file_;
  int check_period_;
};

}  // namespace hdmap
}  // namespace apollo
