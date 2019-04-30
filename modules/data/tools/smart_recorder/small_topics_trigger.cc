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

#include "modules/data/tools/smart_recorder/small_topics_trigger.h"

#include "cyber/common/log.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace data {

SmallTopicsTrigger::SmallTopicsTrigger() {
  trigger_name_ = "SmallTopicsTrigger";
}

bool SmallTopicsTrigger::Init(const SmartRecordTrigger& trigger_conf) {
  // Have to instantiate the wanted classes here that do nothing but
  // register themselves to global factory which then provides reflections later
  apollo::transform::TransformStampeds tf_instance;
  apollo::drivers::gnss::GnssBestPose gnss_instance;
  return TriggerBase::Init(trigger_conf);
}

bool SmallTopicsTrigger::ShouldRestore(const RecordMessage& msg) const {
  return trigger_obj_->enabled() &&
         GetChannelTypes().find(msg.channel_name) != GetChannelTypes().end();
}

}  // namespace data
}  // namespace apollo
