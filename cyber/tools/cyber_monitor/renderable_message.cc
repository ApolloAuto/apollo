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

#include "cyber/tools/cyber_monitor/renderable_message.h"

#include <ncurses.h>

#include "cyber/tools/cyber_monitor/screen.h"

void RenderableMessage::SplitPages(int key) {
  switch (key) {
    case CTRL('d'):
    case KEY_NPAGE:
      ++page_index_;
      if (page_index_ >= pages_) {
        page_index_ = pages_ - 1;
      }
      break;

    case CTRL('u'):
    case KEY_PPAGE:
      --page_index_;
      if (page_index_ < 1) {
        page_index_ = 0;
      }
      break;
    default: {}
  }
}
