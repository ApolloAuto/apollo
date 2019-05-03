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
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace data {

using apollo::common::DriveEvent;

DriveEventTrigger::DriveEventTrigger() { trigger_name_ = "DriveEventTrigger"; }

bool DriveEventTrigger::Init(const SmartRecordTrigger& trigger_conf) {
  // Have to instantiate the wanted classes here that do nothing but
  // register themselves to global factory which then provides reflections later
  apollo::common::DriveEvent drive_event_instance;
  apollo::drivers::CompressedImage image_instance;
  apollo::localization::LocalizationEstimate pose_instance;
  return TriggerBase::Init(trigger_conf);
}

void DriveEventTrigger::Pull(const RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }
  // Simply check the channel
  if (msg.channel_name == FLAGS_drive_event_topic) {
    DriveEvent drive_event_msg;
    drive_event_msg.ParseFromString(msg.content);
    const uint64_t header_time = static_cast<uint64_t>(
        drive_event_msg.header().timestamp_sec() * 1000000000UL);
    AINFO << "drive event trigger is pulled: " << header_time << " - "
          << msg.channel_name;
    TriggerIt(header_time);
  }
}

}  // namespace data
}  // namespace apollo
