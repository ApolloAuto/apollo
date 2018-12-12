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

#include "cyber/transport/shm/condition_notifier.h"

#include <pthread.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <thread>

#include "cyber/common/log.h"
#include "cyber/common/util.h"

namespace apollo {
namespace cyber {
namespace transport {

using common::Hash;

ConditionNotifier::ConditionNotifier() {
  key_ = static_cast<key_t>(Hash("/apollo/cyber/transport/shm/notifier"));
  ADEBUG << "condition notifier key: " << key_;
  shm_size_ = sizeof(Indicator);

  if (!Init()) {
    AERROR << "fail to init condition notifier.";
    is_shutdown_.exchange(true);
  }
}

ConditionNotifier::~ConditionNotifier() { Shutdown(); }

void ConditionNotifier::Shutdown() {
  if (is_shutdown_.exchange(true)) {
    return;
  }

  indicator_->cv.notify_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  Reset();
}

bool ConditionNotifier::Notify(const ReadableInfo& info) {
  if (is_shutdown_.load()) {
    ADEBUG << "notifier is shutdown.";
    return false;
  }

  {
    std::unique_lock<std::mutex> lck(indicator_->mtx);
    auto idx = indicator_->written_info_num % kBufLength;
    indicator_->infos[idx] = info;
    ++indicator_->written_info_num;
  }

  indicator_->cv.notify_all();

  return true;
}

bool ConditionNotifier::Listen(int timeout_ms, ReadableInfo* info) {
  if (info == nullptr) {
    AERROR << "info nullptr.";
    return false;
  }

  if (is_shutdown_.load()) {
    ADEBUG << "notifier is shutdown.";
    return false;
  }

  std::unique_lock<std::mutex> lck(indicator_->mtx);
  if (next_listen_num_ >= indicator_->written_info_num) {
    uint64_t target = next_listen_num_;
    if (!indicator_->cv.wait_for(
            lck, std::chrono::milliseconds(timeout_ms), [target, this]() {
              return this->indicator_->written_info_num > target ||
                     this->is_shutdown_.load();
            })) {
      ADEBUG << "timeout";
      return false;
    }

    if (is_shutdown_.load()) {
      AINFO << "notifier is shutdown.";
      return false;
    }
  }

  if (next_listen_num_ == 0) {
    next_listen_num_ = indicator_->written_info_num - 1;
  }

  auto idx = next_listen_num_ % kBufLength;
  *info = indicator_->infos[idx];
  next_listen_num_ += 1;

  return true;
}

bool ConditionNotifier::Init() { return OpenOrCreate(); }

bool ConditionNotifier::OpenOrCreate() {
  // create managed_shm_
  int retry = 0;
  int shmid = 0;
  while (retry < 2) {
    shmid = shmget(key_, shm_size_, 0644 | IPC_CREAT | IPC_EXCL);
    if (shmid != -1) {
      break;
    }

    if (EINVAL == errno) {
      AINFO << "need larger space, recreate.";
      Reset();
      Remove();
      ++retry;
    } else if (EEXIST == errno) {
      ADEBUG << "shm already exist, open only.";
      return OpenOnly();
    } else {
      break;
    }
  }

  if (shmid == -1) {
    AERROR << "create shm failed, error code: " << strerror(errno);
    return false;
  }

  // attach managed_shm_
  managed_shm_ = shmat(shmid, nullptr, 0);
  if (managed_shm_ == reinterpret_cast<void*>(-1)) {
    AERROR << "attach shm failed.";
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  // create indicator_
  indicator_ = new (managed_shm_) Indicator();
  if (indicator_ == nullptr) {
    AERROR << "create indicator failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  pthread_mutexattr_t mtx_attr;
  pthread_mutexattr_init(&mtx_attr);
  pthread_mutexattr_setpshared(&mtx_attr, PTHREAD_PROCESS_SHARED);
  pthread_mutex_init(indicator_->mtx.native_handle(), &mtx_attr);

  pthread_condattr_t cond_attr;
  pthread_condattr_init(&cond_attr);
  pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
  pthread_cond_init(indicator_->cv.native_handle(), &cond_attr);

  ADEBUG << "open or create true.";
  return true;
}

bool ConditionNotifier::OpenOnly() {
  // get managed_shm_
  int shmid = shmget(key_, 0, 0644);
  if (shmid == -1) {
    AERROR << "get shm failed.";
    return false;
  }

  // attach managed_shm_
  managed_shm_ = shmat(shmid, nullptr, 0);
  if (managed_shm_ == reinterpret_cast<void*>(-1)) {
    AERROR << "attach shm failed.";
    return false;
  }

  // get indicator_
  indicator_ = reinterpret_cast<Indicator*>(managed_shm_);
  if (indicator_ == nullptr) {
    AERROR << "get indicator failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return false;
  }

  ADEBUG << "open true.";
  return true;
}

bool ConditionNotifier::Remove() {
  int shmid = shmget(key_, 0, 0644);
  if (shmid == -1 || shmctl(shmid, IPC_RMID, 0) == -1) {
    AERROR << "remove shm failed, error code: " << strerror(errno);
    return false;
  }
  ADEBUG << "remove success.";

  return true;
}

void ConditionNotifier::Reset() {
  indicator_ = nullptr;
  if (managed_shm_ != nullptr) {
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
  }
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
