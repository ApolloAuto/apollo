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

#ifndef TOOLS_CVT_MONITOR_RENDERABLE_MESSAGE_H_
#define TOOLS_CVT_MONITOR_RENDERABLE_MESSAGE_H_

#include <string>

class Screen;

class RenderableMessage {
 public:
  static constexpr int FrameRatio_Precision = 2;

  explicit RenderableMessage(RenderableMessage* parent = nullptr,
                             int lineNo = 0)
      : line_no_(lineNo),
        pages_(1),
        page_index_(0),
        page_item_count_(24),
        parent_(parent),
        frame_ratio_(0.0) {}

  virtual ~RenderableMessage() { parent_ = nullptr; }

  virtual void Render(const Screen* s, int key) = 0;
  virtual RenderableMessage* Child(int /* lineNo */) const = 0;

  virtual double frame_ratio(void) { return frame_ratio_; }

  RenderableMessage* parent(void) const { return parent_; }
  void set_parent(RenderableMessage* parent) {
    if (parent == parent_) {
      return;
    }
    parent_ = parent;
  }

  int page_item_count(void) const { return page_item_count_; }

 protected:
  int* line_no(void) { return &line_no_; }
  void set_line_no(int lineNo) { line_no_ = lineNo; }
  void reset_line_page(void) {
    line_no_ = 0;
    page_index_ = 0;
  }
  void SplitPages(int key);

  int line_no_;
  int pages_;
  int page_index_;
  int page_item_count_;
  RenderableMessage* parent_;
  double frame_ratio_;

  friend class Screen;
};  // RenderableMessage

#endif  // TOOLS_CVT_MONITOR_RENDERABLE_MESSAGE_H_
