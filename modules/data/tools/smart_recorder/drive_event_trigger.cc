/******************************************************************************
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

#include "modules/data/tools/smart_recorder/drive_event_trigger.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/proto/drive_event.pb.h"

namespace apollo {
namespace data {

using apollo::common::DriveEvent;

DriveEventTrigger::DriveEventTrigger() { trigger_name_ = "DriveEventTrigger"; }

void DriveEventTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }
  // Simply check the channel
  if (msg.channel_name == FLAGS_drive_event_topic) {
    DriveEvent drive_event_msg;
    drive_event_msg.ParseFromString(msg.content);
    const uint64_t header_time = static_cast<uint64_t>(
        SecondsToNanoSeconds(drive_event_msg.header().timestamp_sec()));
    AINFO << "drive event trigger is pulled: " << header_time << " - "
          << msg.channel_name;
    TriggerIt(header_time);
  }
}

}  // namespace data
}  // namespace apollo
