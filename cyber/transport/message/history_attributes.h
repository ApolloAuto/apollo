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

#ifndef CYBER_TRANSPORT_MESSAGE_HISTORY_ATTRIBUTES_H_
#define CYBER_TRANSPORT_MESSAGE_HISTORY_ATTRIBUTES_H_

#include <cstdint>

#include "cyber/proto/qos_profile.pb.h"

namespace apollo {
namespace cyber {
namespace transport {

struct HistoryAttributes {
  HistoryAttributes()
      : history_policy(proto::QosHistoryPolicy::HISTORY_KEEP_LAST),
        depth(1000) {}
  HistoryAttributes(const proto::QosHistoryPolicy& qos_history_policy,
                    uint32_t history_depth)
      : history_policy(qos_history_policy), depth(history_depth) {}

  proto::QosHistoryPolicy history_policy;
  uint32_t depth;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_MESSAGE_HISTORY_ATTRIBUTES_H_
