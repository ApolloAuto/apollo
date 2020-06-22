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
#pragma once

#include <memory>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/storytelling/frame_manager.h"
#include "modules/storytelling/proto/storytelling_config.pb.h"
#include "modules/storytelling/story_tellers/base_teller.h"

namespace apollo {
namespace storytelling {

class Storytelling final : public apollo::cyber::TimerComponent {
 public:
  Storytelling() = default;
  ~Storytelling() = default;

  bool Init() override;

  bool Proc() override;

 private:
  std::vector<std::unique_ptr<BaseTeller>> story_tellers_;
  Stories stories_;
  StorytellingConfig config_;
  std::shared_ptr<FrameManager> frame_manager_;
  std::shared_ptr<::apollo::cyber::Writer<Stories>> story_writer_ = nullptr;
};

CYBER_REGISTER_COMPONENT(Storytelling)
}  // namespace storytelling
}  // namespace apollo
