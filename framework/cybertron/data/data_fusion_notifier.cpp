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
#include "cybertron/data/data_fusion_notifier.h"

#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include "cybertron/data/data_cache.h"

namespace apollo {
namespace cybertron {
namespace data {

void DataFusionNotifier::Notify(uint64_t channel_id) {
  if (!all_has_msg_) {
    if (UpdateChannelStatus()) {
      callback_();
    }
  } else {
    callback_();
  }
}

bool DataFusionNotifier::UpdateChannelStatus() {
  bool all_has_msg = true;
  for (auto& status : channel_status_) {
    if (!status.has_msg) {
      status.has_msg = DataCache::Instance()->HasMessage(status.channel_id);
      all_has_msg &= status.has_msg;
    }
  }
  all_has_msg_ = all_has_msg;
  return all_has_msg_;
}

void DataFusionNotifier::RegisterCallback(
    const std::vector<uint64_t>& channel_vec,
    std::function<void(void)>&& callback) {
  if (channel_vec.empty()) {
    return;
  }

  main_channel_ = channel_vec[0];

  for (int i = 1; i < channel_vec.size(); i++) {
    ChannelStatus status;
    status.channel_id = channel_vec[i];
    status.has_msg = false;
    channel_status_.emplace_back(status);
  }
  callback_ = callback;
}

}  // namespace data
}  // namespace cybertron
}  // namespace apollo
