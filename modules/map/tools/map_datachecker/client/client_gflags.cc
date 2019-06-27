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
#include "modules/map/tools/map_datachecker/client/client_gflags.h"

namespace apollo {
namespace hdmap {
DEFINE_string(stage, "", "data acquisition stage");
DEFINE_string(cmd, "start", "command corresponding to stage");
DEFINE_string(record_path, "", "record path");
DEFINE_string(
    client_conf_yaml,
    "/apollo/modules/map/tools/map_datachecker/client/conf/client.yaml",
    "client configurations");
}  // namespace hdmap
}  // namespace apollo
