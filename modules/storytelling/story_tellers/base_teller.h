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

#include "modules/storytelling/frame_manager.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"
#include "modules/storytelling/proto/storytelling_config.pb.h"

namespace apollo {
namespace storytelling {

class BaseTeller {
 public:
  explicit BaseTeller(const std::shared_ptr<FrameManager>& frame_manager)
      : frame_manager_(frame_manager) {}
  virtual ~BaseTeller() = default;
  virtual void Init(const StorytellingConfig& storytelling_conf) = 0;
  virtual void Update(Stories* stories) = 0;

 protected:
  std::shared_ptr<FrameManager> frame_manager_;
};

}  // namespace storytelling
}  // namespace apollo
