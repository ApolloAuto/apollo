/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#ifndef CYBER_TRANSPORT_RTPS_COMMON_TYPE_H
#define CYBER_TRANSPORT_RTPS_COMMON_TYPE_H

#include <functional>
#include <memory>
#include <string>

#include "cyber/transport/message/message_info.h"
namespace apollo {
namespace cyber {
namespace transport {
namespace rtps {
using subsciber_callback =
    std::function<void(const std::shared_ptr<std::string>& msg_str,
                       uint64_t channel_id, const MessageInfo& msg_info)>;
}  // namespace rtps
}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_RTPS_COMMON_TYPE_H
