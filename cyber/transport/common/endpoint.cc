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

#include "cyber/transport/common/endpoint.h"
#include "cyber/common/global_data.h"

namespace apollo {
namespace cyber {
namespace transport {

Endpoint::Endpoint(const RoleAttributes& attr)
    : enabled_(false), id_(), attr_(attr) {
  if (!attr_.has_host_name()) {
    attr_.set_host_name(common::GlobalData::Instance()->HostName());
  }

  if (!attr_.has_process_id()) {
    attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
  }

  if (!attr_.has_id()) {
    attr_.set_id(id_.HashValue());
  }
}

Endpoint::~Endpoint() {}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
