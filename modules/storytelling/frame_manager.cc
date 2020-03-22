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

#include "modules/storytelling/frame_manager.h"

namespace apollo {
namespace storytelling {

FrameManager::FrameManager()
    : log_buffer_(apollo::common::monitor::MonitorMessageItem::STORYTELLING) {}

void FrameManager::Init(const std::shared_ptr<cyber::Node>& node) {
  node_ = node;
}

void FrameManager::StartFrame() { node_->Observe(); }

void FrameManager::EndFrame() {
  // Print and publish all monitor logs.
  log_buffer_.Publish();
}

}  // namespace storytelling
}  // namespace apollo
