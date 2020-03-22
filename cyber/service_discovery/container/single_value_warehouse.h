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

#ifndef CYBER_SERVICE_DISCOVERY_CONTAINER_SINGLE_VALUE_WAREHOUSE_H_
#define CYBER_SERVICE_DISCOVERY_CONTAINER_SINGLE_VALUE_WAREHOUSE_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/service_discovery/container/warehouse_base.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class SingleValueWarehouse : public WarehouseBase {
 public:
  using RoleMap = std::unordered_map<uint64_t, RolePtr>;

  SingleValueWarehouse() {}
  virtual ~SingleValueWarehouse() {}

  bool Add(uint64_t key, const RolePtr& role,
           bool ignore_if_exist = true) override;

  void Clear() override;
  std::size_t Size() override;

  void Remove(uint64_t key) override;
  void Remove(uint64_t key, const RolePtr& role) override;
  void Remove(const proto::RoleAttributes& target_attr) override;

  bool Search(uint64_t key) override;
  bool Search(uint64_t key, RolePtr* first_matched_role) override;
  bool Search(uint64_t key,
              proto::RoleAttributes* first_matched_role_attr) override;
  bool Search(uint64_t key, std::vector<RolePtr>* matched_roles) override;
  bool Search(uint64_t key,
              std::vector<proto::RoleAttributes>* matched_roles_attr) override;

  bool Search(const proto::RoleAttributes& target_attr) override;
  bool Search(const proto::RoleAttributes& target_attr,
              RolePtr* first_matched) override;
  bool Search(const proto::RoleAttributes& target_attr,
              proto::RoleAttributes* first_matched_role_attr) override;
  bool Search(const proto::RoleAttributes& target_attr,
              std::vector<RolePtr>* matched_roles) override;
  bool Search(const proto::RoleAttributes& target_attr,
              std::vector<proto::RoleAttributes>* matched_roles_attr) override;

  void GetAllRoles(std::vector<RolePtr>* roles) override;
  void GetAllRoles(std::vector<proto::RoleAttributes>* roles_attr) override;

 private:
  RoleMap roles_;
  base::AtomicRWLock rw_lock_;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_CONTAINER_SINGLE_VALUE_WAREHOUSE_H_
