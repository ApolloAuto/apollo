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

#ifndef CYBERTRON_CYBERTRON_H_
#define CYBERTRON_CYBERTRON_H_

#include <string>

#include "cybertron/common/log.h"
#include "cybertron/component/component.h"
#include "cybertron/init.h"
#include "cybertron/node/node.h"
#include "cybertron/scheduler/task.h"
#include "cybertron/time/time.h"
#include "cybertron/timer/timer.h"

#define LOG_DEBUG ADEBUG
#define LOG_INFO AINFO
#define LOG_WARN AWARN
#define LOG_ERROR AERROR

#define XLOG_ERROR(...) printf(__VA_ARGS__);

namespace apollo {
namespace cybertron {

std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space = "");
}
}  // namespace apollo

#endif  // CYBERTRON_CYBERTRON_H_
