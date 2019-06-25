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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_MICRO_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_MICRO_H
#include <grpc++/grpc++.h>
#include <string>
#include <vector>
#include "cyber/cyber.h"
namespace apollo {
namespace hdmap {
#ifndef SYSTEM_OUTPUT_STREAM
#define SYSTEM_OUTPUT_STREAM stderr
#define SYS_STREAM SYSTEM_OUTPUT_STREAM
#endif

#ifndef CLIENT_OUTPUT_STREAM
#define CLIENT_OUTPUT_STREAM stdout
#define USER_STREAM CLIENT_OUTPUT_STREAM
#endif

std::vector<std::string> get_file_lines(const std::string& path);
inline double unixtime_now() { return apollo::cyber::Time::Now().ToSecond(); }
}  // namespace hdmap
}  // namespace apollo
#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_MICRO_H
