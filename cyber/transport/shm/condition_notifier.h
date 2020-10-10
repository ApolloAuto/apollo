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

#ifndef CYBER_TRANSPORT_SHM_CONDITION_NOTIFIER_H_
#define CYBER_TRANSPORT_SHM_CONDITION_NOTIFIER_H_

#include <sys/types.h>
#include <atomic>
#include <cstdint>

#include "cyber/common/macros.h"
#include "cyber/transport/shm/notifier_base.h"

namespace apollo {
namespace cyber {
namespace transport {

const uint32_t kBufLength = 4096;

class ConditionNotifier : public NotifierBase {
  struct Indicator {
    std::atomic<uint64_t> next_seq = {0};
    ReadableInfo infos[kBufLength];
    uint64_t seqs[kBufLength] = {0};
  };

 public:
  virtual ~ConditionNotifier();

  void Shutdown() override;
  bool Notify(const ReadableInfo& info) override;
  bool Listen(int timeout_ms, ReadableInfo* info) override;

  static const char* Type() { return "condition"; }

 private:
  bool Init();
  bool OpenOrCreate();
  bool OpenOnly();
  bool Remove();
  void Reset();

  key_t key_ = 0;
  void* managed_shm_ = nullptr;
  size_t shm_size_ = 0;
  Indicator* indicator_ = nullptr;
  uint64_t next_seq_ = 0;
  std::atomic<bool> is_shutdown_ = {false};

  DECLARE_SINGLETON(ConditionNotifier)
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_SHM_CONDITION_NOTIFIER_H_
