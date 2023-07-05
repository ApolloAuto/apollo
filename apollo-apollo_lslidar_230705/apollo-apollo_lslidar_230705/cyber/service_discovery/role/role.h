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

#ifndef CYBER_SERVICE_DISCOVERY_ROLE_ROLE_H_
#define CYBER_SERVICE_DISCOVERY_ROLE_ROLE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "cyber/proto/role_attributes.pb.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class RoleBase;
using RolePtr = std::shared_ptr<RoleBase>;
using RoleNode = RoleBase;
using RoleNodePtr = std::shared_ptr<RoleNode>;

class RoleWriter;
using RoleWriterPtr = std::shared_ptr<RoleWriter>;
using RoleReader = RoleWriter;
using RoleReaderPtr = std::shared_ptr<RoleReader>;

class RoleServer;
using RoleServerPtr = std::shared_ptr<RoleServer>;
using RoleClient = RoleServer;
using RoleClientPtr = std::shared_ptr<RoleClient>;

class RoleBase {
 public:
  RoleBase();
  explicit RoleBase(const proto::RoleAttributes& attr,
                    uint64_t timestamp_ns = 0);
  virtual ~RoleBase() = default;

  virtual bool Match(const proto::RoleAttributes& target_attr) const;
  bool IsEarlierThan(const RoleBase& other) const;

  const proto::RoleAttributes& attributes() const { return attributes_; }
  void set_attributes(const proto::RoleAttributes& attr) { attributes_ = attr; }

  uint64_t timestamp_ns() const { return timestamp_ns_; }
  void set_timestamp_ns(uint64_t timestamp_ns) { timestamp_ns_ = timestamp_ns; }

 protected:
  proto::RoleAttributes attributes_;
  uint64_t timestamp_ns_;
};

class RoleWriter : public RoleBase {
 public:
  RoleWriter() {}
  explicit RoleWriter(const proto::RoleAttributes& attr,
                      uint64_t timestamp_ns = 0);
  virtual ~RoleWriter() = default;

  bool Match(const proto::RoleAttributes& target_attr) const override;
};

class RoleServer : public RoleBase {
 public:
  RoleServer() {}
  explicit RoleServer(const proto::RoleAttributes& attr,
                      uint64_t timestamp_ns = 0);
  virtual ~RoleServer() = default;

  bool Match(const proto::RoleAttributes& target_attr) const override;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_ROLE_ROLE_H_
