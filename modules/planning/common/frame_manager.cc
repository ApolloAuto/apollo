/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/common/frame_manager.h"

#include <utility>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

std::unique_ptr<Frame> FrameManager::CreateFrame(
    const uint32_t sequence_num) const {
  std::unique_ptr<Frame> frame(new Frame(sequence_num));
  return frame;
}

Frame* FrameManager::GetFrame(const uint32_t sequence_num) {
  return apollo::common::util::FindLinkedPtrOrNull(frames_, sequence_num);
}

Frame* FrameManager::GetLastFrame() {
  if (!sequence_queue_.empty()) {
    return GetFrame(sequence_queue_.back());
  }
  return nullptr;
}

void FrameManager::SaveFrame(std::unique_ptr<Frame>* const frame) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(frame->get());
  sequence_queue_.emplace_back((*frame)->SequenceNum());
  frames_[(*frame)->SequenceNum()] = std::move(*frame);
  if (sequence_queue_.size() >
      static_cast<size_t>(FLAGS_max_history_frame_num)) {
    frames_.erase(sequence_queue_.front());
    sequence_queue_.pop_front();
  }
}

void FrameManager::Clear() {
  while (!sequence_queue_.empty()) {
    frames_.erase(sequence_queue_.front());
    sequence_queue_.pop_front();
  }
}

}  // namespace planning
}  // namespace apollo
