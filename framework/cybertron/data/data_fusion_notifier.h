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

#ifndef CYBERTRON_DATA_DATA_FUSION_NOTIFIER_H_
#define CYBERTRON_DATA_DATA_FUSION_NOTIFIER_H_

#include <atomic>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace apollo {
namespace cybertron {
namespace data {

class DataFusionNotifier {
 public:
  void Notify(uint64_t channel_id);
  bool UpdateChannelStatus();
  void RegisterCallback(const std::vector<uint64_t>& channel_vec,
                        std::function<void()>&& callback);
  uint64_t MainChannel() const { return main_channel_; }

 private:
  struct ChannelStatus {
    uint64_t channel_id;
    bool has_msg;
  };

  uint64_t main_channel_ = 0;
  bool all_has_msg_ = false;
  std::vector<ChannelStatus> channel_status_;

  std::function<void()> callback_;
};

}  // namespace data
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DATA_DATA_FUSION_NOTIFIER_H_
