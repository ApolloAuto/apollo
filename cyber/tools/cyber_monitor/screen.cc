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

#include "screen.h"
#include "cyber_topology_message.h"
#include "general_channel_message.h"
#include "renderable_message.h"

#include <ncurses.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {
constexpr double MinHalfFrameRatio = 12.5;
}

Screen* Screen::Instance(void) {
  static Screen s;
  return &s;
}

const char Screen::InteractiveCmdStr[] =
    "Common Commands for all:\n"
    "   q | Q | Esc -- quit\n"
    "   Backspace -- go back\n"
    "   h | H -- go to show help info\n"
    "\n"
    "Common Commands for Topology and Channel Message:\n"
    "   PgDn | ^d -- show next page\n"
    "   PgUp | ^u -- show previous page\n"
    "\n"
    "   Up Arrow -- move up one line\n"
    "   Down Arrow -- move down one line\n"
    "   Right Arrow -- enter the selected Channel or Repeated Datum\n"
    "   Left Arrow -- go back to the upper level\n"
    "\n"
    "   Enter -- the same with Right Arrow key\n"
    "   a | A -- the same with Left Arrow key\n"
    "   d | D -- the same with Right Arrow key\n"
    "   w | W -- the same with Up Arrow key\n"
    "   s | S -- the same with Down Arrow key\n"
    "\n"
    "Commands for Topology message:\n"
    "   f | F -- show frame ratio for all channel messages\n"
    "   t | T -- show channel message type\n"
    "\n"
    "   Space -- Enable|Disable channel Message\n"
    "\n"
    "Commands for Channel:\n"
    "   i | I -- show Reader and Writers of Channel\n"
    "   b | B -- show Debug String of Channel Message\n"
    "\n"
    "Commands for Channel Repeated Datum:\n"
    "   n | N -- next repeated data item\n"
    "   m | M -- previous repeated data item\n"
    "   , -- enable|disable to show all repeated items\n";

Screen::Screen()
    : current_color_pair_(INVALID),
      canRun_(false),
      current_state_(State::RenderMessage),
      highlight_direction_(0),
      current_render_obj_(nullptr) {}

Screen::~Screen() {
  current_render_obj_ = nullptr;
  endwin();
}

inline bool Screen::IsInit(void) const { return (stdscr != nullptr); }

void Screen::Init(void) {
  initscr();
  if (stdscr == nullptr) {
    return;
  }
  nodelay(stdscr, true);
  keypad(stdscr, true);
  meta(stdscr, true);
  curs_set(0);
  noecho();

  bkgd(COLOR_BLACK);

  start_color();
  init_pair(GREEN_BLACK, COLOR_GREEN, COLOR_BLACK);
  init_pair(YELLOW_BLACK, COLOR_YELLOW, COLOR_BLACK);
  init_pair(RED_BLACK, COLOR_RED, COLOR_BLACK);
  init_pair(WHITE_BLACK, COLOR_WHITE, COLOR_BLACK);
  init_pair(BLACK_WHITE, COLOR_BLACK, COLOR_WHITE);

  refresh();
  clear();

  canRun_ = true;
}

int Screen::Width(void) const { return COLS; }

int Screen::Height(void) const { return LINES; }

void Screen::SetCurrentColor(ColorPair color) const {
  if (color == INVALID) {
    return;
  }
  if (IsInit()) {
    current_color_pair_ = color;
    attron(COLOR_PAIR(color));
  }
}
void Screen::AddStr(int x, int y, const char* str) const {
  if (IsInit()) {
    mvaddstr(y, x, str);
  }
}

void Screen::AddStr(const char* str) const {
  if (IsInit()) {
    addstr(str);
  }
}

void Screen::ClearCurrentColor(void) const {
  if (IsInit()) {
    attroff(COLOR_PAIR(current_color_pair_));
    current_color_pair_ = INVALID;
  }
}

void Screen::AddStr(int x, int y, ColorPair color, const char* str) const {
  if (IsInit()) {
    attron(COLOR_PAIR(color));
    mvaddstr(y, x, str);
    attroff(COLOR_PAIR(color));
  }
}

void Screen::MoveOffsetXY(int offsetX, int offsetY) const {
  if (IsInit()) {
    int x, y;
    getyx(stdscr, y, x);
    move(y + offsetY, x + offsetX);
  }
}

void Screen::HighlightLine(int lineNo) {
  if (IsInit() && lineNo < Height()) {
    SetCurrentColor(WHITE_BLACK);
    for (int x = 0; x < Width(); ++x) {
      chtype ch = mvinch(lineNo + highlight_direction_, x);
      ch &= A_CHARTEXT;
      if (ch == ' ') {
        mvaddch(lineNo + highlight_direction_, x, ch);
      }
    }
    ClearCurrentColor();

    SetCurrentColor(BLACK_WHITE);
    for (int x = 0; x < Width(); ++x) {
      chtype ch = mvinch(lineNo, x);
      mvaddch(lineNo, x, ch & A_CHARTEXT);
    }
    ClearCurrentColor();
  }
}

int Screen::SwitchState(int ch) {
  switch (current_state_) {
    case State::RenderInterCmdInfo:
      if (KEY_BACKSPACE == ch) {
        current_state_ = State::RenderMessage;
        clear();
        ch = 27;
      }
      break;
    case State::RenderMessage:
      if ('h' == ch || 'H' == ch) {
        current_state_ = State::RenderInterCmdInfo;
        clear();
      }
      break;
    default: {}
  }
  return ch;
}

void Screen::Run() {
  if (stdscr == nullptr || current_render_obj_ == nullptr) {
    return;
  }

  highlight_direction_ = 0;

  void (Screen::*showFuncs[])(int) = {&Screen::ShowRenderMessage,
                                      &Screen::ShowInteractiveCmd};

  do {
    int ch = getch();

    if (ch == 'q' || ch == 'Q' || ch == 27) {
      canRun_ = false;
      break;
    }

    ch = SwitchState(ch);

    (this->*showFuncs[static_cast<int>(current_state_)])(ch);

    double fr = current_render_obj_->frame_ratio();
    if (fr < MinHalfFrameRatio) {
      fr = MinHalfFrameRatio;
    }
    int period = static_cast<int>(1000.0 / fr);
    period >>= 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(period));
  } while (canRun_);
}

void Screen::Resize(void) {
  if (IsInit()) {
    clear();
    refresh();
  }
}

void Screen::ShowRenderMessage(int ch) {
  erase();
  current_render_obj_->Render(this, ch);

  int* y = current_render_obj_->line_no();

  HighlightLine(*y);
  move(*y, 0);
  refresh();

  switch (ch) {
    case 's':
    case 'S':
    case KEY_DOWN:
      ++(*y);
      highlight_direction_ = -1;
      if (*y >= Height()) {
        *y = Height() - 1;
      }
      break;

    case 'w':
    case 'W':
    case KEY_UP:
      --(*y);
      if (*y < 1) {
        *y = 1;
      }
      highlight_direction_ = 1;
      if (*y < 0) {
        *y = 0;
      }
      break;

    case 'a':
    case 'A':
    case KEY_BACKSPACE:
    case KEY_LEFT: {
      RenderableMessage* p = current_render_obj_->parent();
      if (p) {
        current_render_obj_ = p;
        y = p->line_no();
        clear();
      }
      break;
    }

    case '\n':
    case '\r':
    case 'd':
    case 'D':
    case KEY_RIGHT: {
      RenderableMessage* child = current_render_obj_->Child(*y);

      if (child) {
        child->reset_line_page();
        current_render_obj_ = child;
        y = child->line_no();
        clear();
      }
      break;
    }
  }
}

void Screen::ShowInteractiveCmd(int) {
  unsigned y = 0;

  SetCurrentColor(Screen::WHITE_BLACK);
  AddStr((Width() - 19) / 2, y++, "Interactive Command");

  const char* ptr = InteractiveCmdStr;
  while (*ptr != '\0') {
    const char* sub = std::strchr(ptr, '\n');
    std::string subStr(ptr, sub);
    AddStr(0, y++, subStr.c_str());
    ptr = sub + 1;
  }

  ClearCurrentColor();
}
