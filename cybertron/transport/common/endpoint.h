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

#ifndef CYBERTRON_TRANSPORT_COMMON_ENDPOINT_H_
#define CYBERTRON_TRANSPORT_COMMON_ENDPOINT_H_

#include <memory>
#include <string>

#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/transport/common/identity.h"

namespace apollo {
namespace cybertron {
namespace transport {

class Endpoint;
using EndpointPtr = std::shared_ptr<Endpoint>;

using proto::RoleAttributes;

class Endpoint {
 public:
  explicit Endpoint(const RoleAttributes& attr);
  virtual ~Endpoint();

  const Identity& id() const { return id_; }
  const RoleAttributes& attributes() const { return attr_; }

 protected:
  bool enabled_;
  Identity id_;
  RoleAttributes attr_;
};

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_COMMON_ENDPOINT_H_
