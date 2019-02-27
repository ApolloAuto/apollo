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

#ifndef MODULES_ULTANALYSE_ULTANALYSE_GFLAGS_H_
#define MODULES_ULTANALYSE_ULTANALYSE_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(node_name);
DECLARE_string(module_name);
DECLARE_string(adapter_config_filename);
DECLARE_bool(ultanalyse_enabled);
DECLARE_double(min_sonar_range_detectable);
DECLARE_double(max_sonar_range_detectable);

#endif
