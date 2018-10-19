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

#include "cyber/common/types.h"
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

using apollo::cyber::proto::RoleAttributes;

class RoleBase {
 public:
  RoleBase();
  explicit RoleBase(const RoleAttributes& attr, uint64_t timestamp = 0);
  virtual ~RoleBase();

  virtual bool Match(const RoleAttributes& target_attr) const;
  bool IsEarlierThan(const RoleBase& other) const;

  uint64_t timestamp() const { return timestamp_; }
  void set_timestamp(uint64_t timestamp) { timestamp_ = timestamp; }

  // getter and setter
  const RoleAttributes& attributes() const { return attributes_; }
  void set_attributes(const RoleAttributes& attr) { attributes_ = attr; }

 protected:
  RoleAttributes attributes_;
  uint64_t timestamp_;
};

class RoleWriter : public RoleBase {
 public:
  RoleWriter();
  explicit RoleWriter(const RoleAttributes& attr, uint64_t timestamp = 0);
  virtual ~RoleWriter();

  bool Match(const RoleAttributes& target_attr) const override;
};

class RoleServer : public RoleBase {
 public:
  RoleServer();
  explicit RoleServer(const RoleAttributes& attr, uint64_t timestamp = 0);
  virtual ~RoleServer();

  bool Match(const RoleAttributes& target_attr) const override;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_ROLE_ROLE_H_
