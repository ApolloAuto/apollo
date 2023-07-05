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

#ifndef CYBER_CYBER_H_
#define CYBER_CYBER_H_

#include <memory>
#include <string>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/init.h"
#include "cyber/node/node.h"
#include "cyber/task/task.h"
#include "cyber/time/time.h"
#include "cyber/timer/timer.h"

namespace apollo {
namespace cyber {

std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space = "");

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_CYBER_H_
