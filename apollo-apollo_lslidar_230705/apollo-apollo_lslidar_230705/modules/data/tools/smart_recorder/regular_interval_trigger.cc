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

#include "modules/data/tools/smart_recorder/regular_interval_trigger.h"

#include "cyber/common/log.h"

namespace apollo {
namespace data {

RegularIntervalTrigger::RegularIntervalTrigger() {
  trigger_name_ = "RegularIntervalTrigger";
}

void RegularIntervalTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }
  if (current_recording_time_ == 0) {
    current_recording_time_ = msg.time;
    return;
  }
  if (msg.time - current_recording_time_ >
      SecondsToNanoSeconds(recording_interval_)) {
    current_recording_time_ = msg.time;
    AINFO << "regular interval trigger is pulled: " << msg.time;
    TriggerIt(msg.time);
  }
}

}  // namespace data
}  // namespace apollo
