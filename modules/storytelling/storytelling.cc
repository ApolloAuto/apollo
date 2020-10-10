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
#include "modules/storytelling/storytelling.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/storytelling/story_tellers/close_to_junction_teller.h"

namespace apollo {
namespace storytelling {

bool Storytelling::Init() {
  frame_manager_ = std::make_shared<FrameManager>(node_);
  story_tellers_.emplace_back(new CloseToJunctionTeller(frame_manager_));

  if (!cyber::ComponentBase::GetProtoConfig(&config_)) {
    AERROR << "Unable to load storytelling conf file: "
           << cyber::ComponentBase::ConfigFilePath();
    return false;
  }

  story_writer_ =
      node_->CreateWriter<Stories>(config_.topic_config().storytelling_topic());

  // Init all tellers.
  for (const auto& teller : story_tellers_) {
    teller->Init(config_);
  }

  return true;
}

bool Storytelling::Proc() {
  frame_manager_->StartFrame();

  // Query all tellers.
  for (const auto& teller : story_tellers_) {
    teller->Update(&stories_);
  }

  // Send stories.
  apollo::common::util::FillHeader("Storytelling", &stories_);
  story_writer_->Write(stories_);

  frame_manager_->EndFrame();
  return true;
}

}  // namespace storytelling
}  // namespace apollo
