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

#ifndef CYBER_SERVICE_DISCOVERY_CONTAINER_WAREHOUSE_BASE_H_
#define CYBER_SERVICE_DISCOVERY_CONTAINER_WAREHOUSE_BASE_H_

#include <cstdint>
#include <vector>

#include "cyber/service_discovery/role/role.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class WarehouseBase {
 public:
  WarehouseBase() {}
  virtual ~WarehouseBase() {}

  virtual bool Add(uint64_t key, const RolePtr& role, bool ignore_if_exist) = 0;

  virtual void Clear() = 0;
  virtual std::size_t Size() = 0;

  virtual void Remove(uint64_t key) = 0;
  virtual void Remove(uint64_t key, const RolePtr& role) = 0;
  virtual void Remove(const proto::RoleAttributes& target_attr) = 0;

  virtual bool Search(uint64_t key) = 0;
  virtual bool Search(uint64_t key, RolePtr* first_matched_role) = 0;
  virtual bool Search(uint64_t key,
                      proto::RoleAttributes* first_matched_role_attr) = 0;
  virtual bool Search(uint64_t key, std::vector<RolePtr>* matched_roles) = 0;
  virtual bool Search(
      uint64_t key, std::vector<proto::RoleAttributes>* matched_roles_attr) = 0;

  virtual bool Search(const proto::RoleAttributes& target_attr) = 0;
  virtual bool Search(const proto::RoleAttributes& target_attr,
                      RolePtr* first_matched) = 0;
  virtual bool Search(const proto::RoleAttributes& target_attr,
                      proto::RoleAttributes* first_matched_role_attr) = 0;
  virtual bool Search(const proto::RoleAttributes& target_attr,
                      std::vector<RolePtr>* matched_roles) = 0;
  virtual bool Search(
      const proto::RoleAttributes& target_attr,
      std::vector<proto::RoleAttributes>* matched_roles_attr) = 0;

  virtual void GetAllRoles(std::vector<RolePtr>* roles) = 0;
  virtual void GetAllRoles(std::vector<proto::RoleAttributes>* roles_attr) = 0;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_CONTAINER_WAREHOUSE_BASE_H_
