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

#include "modules/map/relative_map/common/relative_map_gflags.h"

DEFINE_string(relative_map_adapter_config_filename,
              "modules/map/relative_map/conf/adapter.conf",
              "gflags conf file for relative map");
DEFINE_string(relative_map_config_filename,
              "modules/map/relative_map/conf/relative_map_config.pb.txt",
              "Relative map configuration file");
