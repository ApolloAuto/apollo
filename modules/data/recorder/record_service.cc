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

#include "modules/data/common/data_gflags.h"
#include "modules/data/recorder/record_service.h"

#include "gflags/gflags.h"
#include "cyber/common/log.h"

namespace apollo {
namespace data {

RecordService::RecordService() = default;

bool RecordService::Init(
    const std::shared_ptr<apollo::cyber::Node>& node) {
  AINFO << "RecordService::Init(), starting...";

  Instance()->server_ = node->CreateService<RecordRequest, RecordResponse>(
      FLAGS_data_record_service_name,
      RecordService::OnRecordRequest);

  CHECK(Instance()->server_ != nullptr);

  AINFO << "RecordService::Init(), service started";

  return true;
}

void RecordService::OnRecordRequest(
    const std::shared_ptr<RecordRequest> &request,
    const std::shared_ptr<RecordResponse> &response) {
  ADEBUG << "Received data record request: " << request->DebugString();
  // TODO(michael): do recording
  response->set_record_result(RecordResponse::PASS);
}

}  // namespace data
}  // namespace apollo
