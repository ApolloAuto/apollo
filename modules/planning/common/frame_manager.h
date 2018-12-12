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

#ifndef MODULES_PLANNING_COMMON_FRAME_MANAGER_H_
#define MODULES_PLANNING_COMMON_FRAME_MANAGER_H_

#include <list>
#include <memory>
#include <unordered_map>

#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

class FrameManager final {
 public:
  FrameManager() = default;
  std::unique_ptr<Frame> CreateFrame(const uint32_t sequence_num) const;
  Frame* GetFrame(const uint32_t sequence_num);
  Frame* GetLastFrame();
  void SaveFrame(std::unique_ptr<Frame>* const frame);
  void Clear();
 private:
  std::unordered_map<uint32_t, std::unique_ptr<Frame>> frames_;
  std::list<uint32_t> sequence_queue_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_MANAGER_H_

