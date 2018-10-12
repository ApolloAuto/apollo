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

#ifndef TOOLS_CVT_MONITOR_SCREEN_H_
#define TOOLS_CVT_MONITOR_SCREEN_H_

#include <map>
#include <string>
#include <vector>

#ifndef CTRL
#define CTRL(c) ((c)&0x1F)
#endif

class RenderableMessage;

class Screen final {
 public:
  static const char InteractiveCmdStr[];

  enum ColorPair {  // foreground color - background color
    INVALID = 0,
    GREEN_BLACK = 1,
    YELLOW_BLACK,
    RED_BLACK,
    WHITE_BLACK,
    BLACK_WHITE
  };
  static Screen* Instance(void);

  ~Screen(void);

  void Init(void);
  void Run(void);
  void Resize();
  void Stop(void) { canRun_ = false; }

  int Width(void) const;
  int Height(void) const;

  void AddStr(int x, int y, ColorPair color, const char* cStr) const;

  ColorPair Color(void) const { return current_color_pair_; }
  void SetCurrentColor(ColorPair color) const;
  void AddStr(int x, int y, const char* str) const;
  void AddStr(const char* str) const;
  void MoveOffsetXY(int offsetX, int offsetY) const;
  void ClearCurrentColor(void) const;

  void SetCurrentRenderMessage(RenderableMessage* const renderObj) {
    if (renderObj) {
      current_render_obj_ = renderObj;
    }
  }

 private:
  explicit Screen();
  Screen(const Screen&) = delete;
  Screen& operator=(const Screen&) = delete;

  int SwitchState(int ch);
  void HighlightLine(int lineNo);

  void ShowInteractiveCmd(int ch);
  void ShowRenderMessage(int ch);

  bool IsInit(void) const;

  enum class State { RenderMessage, RenderInterCmdInfo };

  mutable ColorPair current_color_pair_;
  bool canRun_;
  State current_state_;
  int highlight_direction_;
  RenderableMessage* current_render_obj_;
};

#endif  // TOOLS_CVT_MONITOR_SCREEN_H_
