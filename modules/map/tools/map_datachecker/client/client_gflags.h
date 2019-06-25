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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_GFLAGS_H
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_GFLAGS_H

#include "gflags/gflags.h"

namespace apollo {
namespace hdmap {
DECLARE_string(stage);
DECLARE_string(cmd);
DECLARE_string(record_path);
DECLARE_string(client_conf_yaml);
}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_CLIENT_CLIENT_GFLAGS_H