/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/profiler/block_manager.h"

#include "cyber/croutine/croutine.h"

namespace apollo {
namespace cyber {
namespace profiler {

thread_local std::unordered_map<std::string, Frame>
    BlockManager::routine_frame_map_{};

BlockManager::BlockManager() {}

void BlockManager::StartBlock(Block* block) {
  Frame* frame_ptr = GetRoutineFrame();
  if (frame_ptr == nullptr || block == nullptr)
    return;
  frame_ptr->Push(block);
  block->set_depth(frame_ptr->size());
  block->Start();
}

void BlockManager::EndBlock() {
  Frame* frame_ptr = GetRoutineFrame();
  if (frame_ptr == nullptr || frame_ptr->finished())
    return;

  Block* block = frame_ptr->Top();
  block->End();
  frame_ptr->Pop();

  if (frame_ptr->finished()) {
    const std::string& routine_name = GetRoutineName();
    frame_ptr->DumpToFile(routine_name);
    frame_ptr->Clear();
  }
}

std::string BlockManager::GetRoutineName() {
  std::string routine_name("default_croutine");
  if (croutine::CRoutine::GetCurrentRoutine() != nullptr) {
    routine_name = croutine::CRoutine::GetCurrentRoutine()->name();
  }
  return routine_name;
}

Frame* BlockManager::GetRoutineFrame() {
  const std::string& routine_name = GetRoutineName();
  return &routine_frame_map_[routine_name];
}

}  // namespace profiler
}  // namespace cyber
}  // namespace apollo
