/******************************************************************************der
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>
#include "modules/bridge/common/bridge_proto_diserialized_buf.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace bridge {

class ProtoDiserializedBufBaseFactory {
 public:
  static std::shared_ptr<ProtoDiserializedBufBase> CreateObj(
      const BridgeHeader &header) {
    std::shared_ptr<ProtoDiserializedBufBase> obj;
    if (strcmp("Chassis", header.GetMsgName().c_str()) == 0) {
      obj = std::make_shared<BridgeProtoDiserializedBuf<canbus::Chassis>>(
          FLAGS_chassis_topic);
    }
    return obj;
  }
};

}  // namespace bridge
}  // namespace apollo
