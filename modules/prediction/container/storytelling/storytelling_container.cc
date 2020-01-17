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

#include "modules/prediction/container/storytelling/storytelling_container.h"

namespace apollo {
namespace prediction {

using apollo::storytelling::Stories;

void StoryTellingContainer::Insert(const ::google::protobuf::Message& message) {
  Stories story_message = dynamic_cast<const Stories&>(message);
  close_to_junction_.CopyFrom(story_message.close_to_junction());
}

std::shared_ptr<const hdmap::JunctionInfo> StoryTellingContainer::ADCJunction()
    const {
  std::string adc_junction_id = close_to_junction_.junction_id();
  return PredictionMap::JunctionById(adc_junction_id);
}

const std::string& StoryTellingContainer::ADCJunctionId() const {
  return close_to_junction_.junction_id();
}

double StoryTellingContainer::ADCDistanceToJunction() const {
  return close_to_junction_.distance();
}

}  // namespace prediction
}  // namespace apollo
