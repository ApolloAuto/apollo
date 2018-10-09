/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 * @brief Console color definitions
 */

#pragma once

#include <cstdio>

namespace apollo {
namespace common {
namespace color {

constexpr char ANSI_RED[] = "\x1b[31m";
constexpr char ANSI_GREEN[] = "\x1b[32m";
constexpr char ANSI_YELLOW[] = "\x1b[33m";
constexpr char ANSI_BLUE[] = "\x1b[34m";
constexpr char ANSI_MAGENTA[] = "\x1b[35m";
constexpr char ANSI_CYAN[] = "\x1b[36m";
constexpr char ANSI_RESET[] = "\x1b[0m";

}  // namespace color
}  // namespace common
}  // namespace apollo
