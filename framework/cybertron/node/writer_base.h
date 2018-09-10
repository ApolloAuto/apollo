/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBERTRON_NODE_WRITER_BASE_H_
#define CYBERTRON_NODE_WRITER_BASE_H_

#include <atomic>
#include <string>

#include "cybertron/common/global_data.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/transport/common/identity.h"

namespace apollo {
namespace cybertron {

class WriterBase {
 public:
  explicit WriterBase(const proto::RoleAttributes& role_attr)
      : role_attr_(role_attr), init_(false) {
    if (!role_attr_.has_host_name()) {
      role_attr_.set_host_name(common::GlobalData::Instance()->HostName());
    }
    if (!role_attr_.has_process_id()) {
      role_attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
    }
  }
  virtual ~WriterBase() {}

  virtual bool Init() = 0;
  virtual void Shutdown() = 0;
  const std::string& GetChannelName() const {
    return role_attr_.channel_name();
  }

  bool inited() const { return init_.load(); }

 protected:
  proto::RoleAttributes role_attr_;
  std::atomic<bool> init_;
};

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_NODE_WRITER_BASE_H_
