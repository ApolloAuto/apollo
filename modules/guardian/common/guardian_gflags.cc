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

#include "modules/guardian/common/guardian_gflags.h"

DEFINE_string(node_name, "guardian", "The chassis module name in proto");
DEFINE_string(module_name, "guardian", "Module name");

DEFINE_string(adapter_config_filename, "", "Path for adapter configuration");

DEFINE_double(guardian_cmd_freq, 10, "timer frequency.");

DEFINE_double(guardian_cmd_soft_stop_percentage, 25,
              "Soft stop perceptage when safe mode triggered");

DEFINE_double(guardian_cmd_emergency_stop_percentage, 50,
              "Emergency stop perceptage when safe mode triggered");

DEFINE_bool(guardian_enabled, false,
            "Enable guardian, safe mode activation enabled");
