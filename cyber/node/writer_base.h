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

#ifndef CYBER_NODE_WRITER_BASE_H_
#define CYBER_NODE_WRITER_BASE_H_

#include <atomic>
#include <string>
#include <vector>

#include "cyber/proto/role_attributes.pb.h"

namespace apollo {
namespace cyber {

class WriterBase {
 public:
  explicit WriterBase(const proto::RoleAttributes& role_attr)
      : role_attr_(role_attr), init_(false) {}
  virtual ~WriterBase() {}

  virtual bool Init() = 0;
  virtual void Shutdown() = 0;

  virtual bool HasReader() { return false; }
  virtual void GetReaders(std::vector<proto::RoleAttributes>* readers) {}

  const std::string& GetChannelName() const {
    return role_attr_.channel_name();
  }

  bool IsInit() const { std::lock_guard<std::mutex> g(lock_); return init_; }

 protected:
  proto::RoleAttributes role_attr_;
  mutable std::mutex lock_;
  bool init_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_NODE_WRITER_BASE_H_
