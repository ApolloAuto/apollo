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

#include "modules/tools/navi_generator/backend/hmi/topics_service.h"

#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"

namespace apollo {
namespace navi_generator {

using apollo::common::time::Clock;
using apollo::common::time::millis;
using apollo::common::time::ToSecond;
using apollo::common::util::GetProtoFromFile;
using Json = nlohmann::json;
using google::protobuf::util::MessageToJsonString;

TopicsService::TopicsService(NaviGeneratorWebSocket *websocket)
    : websocket_(websocket) {}

void TopicsService::Update() {
  // TODO(*): Update status
}
}  // namespace navi_generator
}  // namespace apollo
