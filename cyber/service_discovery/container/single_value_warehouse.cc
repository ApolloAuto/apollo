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

#include "cyber/service_discovery/container/single_value_warehouse.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

using base::AtomicRWLock;
using base::ReadLockGuard;
using base::WriteLockGuard;
using proto::RoleAttributes;

bool SingleValueWarehouse::Add(uint64_t key, const RolePtr& role,
                               bool ignore_if_exist) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  if (!ignore_if_exist) {
    if (roles_.find(key) != roles_.end()) {
      return false;
    }
  }
  roles_[key] = role;
  return true;
}

void SingleValueWarehouse::Clear() {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  roles_.clear();
}

std::size_t SingleValueWarehouse::Size() {
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  return roles_.size();
}

void SingleValueWarehouse::Remove(uint64_t key) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  roles_.erase(key);
}

void SingleValueWarehouse::Remove(uint64_t key, const RolePtr& role) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  auto search = roles_.find(key);
  if (search == roles_.end()) {
    return;
  }
  if (!search->second->Match(role->attributes())) {
    return;
  }
  roles_.erase(search);
}

void SingleValueWarehouse::Remove(const RoleAttributes& target_attr) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  for (auto it = roles_.begin(); it != roles_.end();) {
    auto curr_role = it->second;
    if (curr_role->Match(target_attr)) {
      it = roles_.erase(it);
    } else {
      ++it;
    }
  }
}

bool SingleValueWarehouse::Search(uint64_t key) {
  RolePtr role;
  return Search(key, &role);
}

bool SingleValueWarehouse::Search(uint64_t key, RolePtr* first_matched_role) {
  RETURN_VAL_IF_NULL(first_matched_role, false);
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  auto search = roles_.find(key);
  if (search == roles_.end()) {
    return false;
  }
  *first_matched_role = search->second;
  return true;
}

bool SingleValueWarehouse::Search(uint64_t key,
                                  RoleAttributes* first_matched_role_attr) {
  RETURN_VAL_IF_NULL(first_matched_role_attr, false);
  RolePtr role;
  if (!Search(key, &role)) {
    return false;
  }
  first_matched_role_attr->CopyFrom(role->attributes());
  return true;
}

bool SingleValueWarehouse::Search(uint64_t key,
                                  std::vector<RolePtr>* matched_roles) {
  RETURN_VAL_IF_NULL(matched_roles, false);
  RolePtr role;
  if (!Search(key, &role)) {
    return false;
  }
  matched_roles->emplace_back(role);
  return true;
}

bool SingleValueWarehouse::Search(
    uint64_t key, std::vector<RoleAttributes>* matched_roles_attr) {
  RETURN_VAL_IF_NULL(matched_roles_attr, false);
  RolePtr role;
  if (!Search(key, &role)) {
    return false;
  }
  matched_roles_attr->emplace_back(role->attributes());
  return true;
}

bool SingleValueWarehouse::Search(const RoleAttributes& target_attr) {
  RolePtr role;
  return Search(target_attr, &role);
}

bool SingleValueWarehouse::Search(const RoleAttributes& target_attr,
                                  RolePtr* first_matched_role) {
  RETURN_VAL_IF_NULL(first_matched_role, false);
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  for (auto& item : roles_) {
    if (item.second->Match(target_attr)) {
      *first_matched_role = item.second;
      return true;
    }
  }
  return false;
}

bool SingleValueWarehouse::Search(const RoleAttributes& target_attr,
                                  RoleAttributes* first_matched_role_attr) {
  RETURN_VAL_IF_NULL(first_matched_role_attr, false);
  RolePtr role;
  if (!Search(target_attr, &role)) {
    return false;
  }
  first_matched_role_attr->CopyFrom(role->attributes());
  return true;
}

bool SingleValueWarehouse::Search(const RoleAttributes& target_attr,
                                  std::vector<RolePtr>* matched_roles) {
  RETURN_VAL_IF_NULL(matched_roles, false);
  bool find = false;
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  for (auto& item : roles_) {
    if (item.second->Match(target_attr)) {
      matched_roles->emplace_back(item.second);
      find = true;
    }
  }
  return find;
}

bool SingleValueWarehouse::Search(
    const RoleAttributes& target_attr,
    std::vector<RoleAttributes>* matched_roles_attr) {
  RETURN_VAL_IF_NULL(matched_roles_attr, false);
  bool find = false;
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  for (auto& item : roles_) {
    if (item.second->Match(target_attr)) {
      matched_roles_attr->emplace_back(item.second->attributes());
      find = true;
    }
  }
  return find;
}

void SingleValueWarehouse::GetAllRoles(std::vector<RolePtr>* roles) {
  RETURN_IF_NULL(roles);
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  for (auto& item : roles_) {
    roles->emplace_back(item.second);
  }
}

void SingleValueWarehouse::GetAllRoles(
    std::vector<RoleAttributes>* roles_attr) {
  RETURN_IF_NULL(roles_attr);
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  for (auto& item : roles_) {
    roles_attr->emplace_back(item.second->attributes());
  }
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
